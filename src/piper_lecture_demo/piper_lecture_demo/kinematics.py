"""URDF 하나만 읽어서 FK 와 기하학적 Jacobian 을 만든다.

의존성은 numpy 와 파이썬 표준 라이브러리뿐이다. MoveIt 도 KDL 도 쓰지 않는다 —
이 파일에 적힌 것이 곧 강의에서 다룬 두 수식이기 때문이다.

    ① T₀ₙ = T₁T₂⋯Tₙ            (형상)  -> Chain.fk()
    ② v = J(q) q̇                (운동)  -> Chain.jacobian()

회전관절 하나가 Jacobian 의 열 하나이고, 그 열은

    Jᵢ = [ zᵢ × (pₑ − pᵢ) ; zᵢ ]

이다. zᵢ 와 pᵢ 는 ⁰Tᵢ 에서 그냥 꺼내는 값이라, ②는 ①의 부산물로 나온다.
"""

import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_matrix(rpy):
    """URDF 의 고정축 roll-pitch-yaw -> 3x3 회전행렬. R = Rz(y) Ry(p) Rx(r)."""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def axis_angle_to_matrix(axis, angle):
    """Rodrigues 공식. 회전관절 하나가 만드는 회전."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0.0, -a[2], a[1]],
                  [a[2], 0.0, -a[0]],
                  [-a[1], a[0], 0.0]])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def matrix_to_quaternion(R):
    """3x3 회전행렬 -> (x, y, z, w). Marker 자세를 채울 때 쓴다."""
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


class Joint:
    __slots__ = ("name", "type", "parent", "child", "origin", "axis", "lower", "upper")

    def __init__(self, elem):
        self.name = elem.get("name")
        self.type = elem.get("type")
        self.parent = elem.find("parent").get("link")
        self.child = elem.find("child").get("link")

        origin = elem.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.get("xyz"):
                xyz = [float(v) for v in origin.get("xyz").split()]
            if origin.get("rpy"):
                rpy = [float(v) for v in origin.get("rpy").split()]
        self.origin = np.eye(4)
        self.origin[:3, :3] = rpy_to_matrix(rpy)
        self.origin[:3, 3] = xyz

        axis = elem.find("axis")
        self.axis = np.array([float(v) for v in axis.get("xyz").split()]) if axis is not None \
            else np.array([0.0, 0.0, 1.0])

        limit = elem.find("limit")
        if limit is not None and limit.get("lower") is not None:
            self.lower = float(limit.get("lower"))
            self.upper = float(limit.get("upper"))
        else:
            self.lower = self.upper = 0.0

    @property
    def movable(self):
        return self.type in ("revolute", "continuous", "prismatic")


class Chain:
    """base_link 에서 tip 까지의 직렬 사슬."""

    def __init__(self, urdf_path, base="base_link", tip="link6", tool=None):
        """tool 은 tip 프레임에서 잰 공구 끝점(TCP) 오프셋이다 (3-벡터 또는 4x4).

        기준점을 옮기면 FK 뿐 아니라 Jacobian 도 바뀐다 — J 의 선속도 열이
        zi x (pe - pi) 이고 pe 가 바로 이 점이기 때문이다. 플랜지에서 재느냐
        손끝에서 재느냐에 따라 「얼마나 잘 움직이나」의 답이 달라진다.
        """
        root = ET.parse(urdf_path).getroot()
        joints = [Joint(j) for j in root.findall("joint")]
        by_child = {j.child: j for j in joints}

        # tip 에서 base 까지 부모를 거슬러 올라간 뒤 뒤집는다.
        chain, link = [], tip
        while link != base:
            if link not in by_child:
                raise ValueError(f"'{link}' 의 부모 관절을 URDF 에서 찾지 못했다. base='{base}' 확인 필요")
            j = by_child[link]
            chain.append(j)
            link = j.parent
        chain.reverse()

        self.base = base
        self.tip = tip
        self.tool = np.eye(4)
        if tool is not None:
            tool = np.asarray(tool, dtype=float)
            if tool.shape == (3,):
                self.tool[:3, 3] = tool
            elif tool.shape == (4, 4):
                self.tool = tool
            else:
                raise ValueError("tool 은 3-벡터이거나 4x4 여야 한다")
        self.joints = chain
        self.movable = [j for j in chain if j.movable]
        self.names = [j.name for j in self.movable]
        self.lower = np.array([j.lower for j in self.movable])
        self.upper = np.array([j.upper for j in self.movable])

    @property
    def dof(self):
        return len(self.movable)

    def frames(self, q):
        """관절값 q 에 대해 ⁰T 들을 차례로 만든다.

        반환 (T_tip, axes, origins):
          T_tip   — ⁰T_tip · T_tool (4x4). tool 을 줬으면 TCP 까지 간 변환이다
          axes    — 각 가동관절 축의 월드 방향 zᵢ
          origins — 각 가동관절 원점의 월드 위치 pᵢ
        """
        q = np.asarray(q, dtype=float)
        T = np.eye(4)
        axes, origins = [], []
        k = 0
        for j in self.joints:
            T = T @ j.origin                      # 고정 오프셋 (URDF origin)
            if j.movable:
                # 관절 축은 이 프레임 원점을 지난다. 관절을 돌려도 원점은 그대로다.
                axes.append(T[:3, :3] @ j.axis)
                origins.append(T[:3, 3].copy())
                move = np.eye(4)
                if j.type == "prismatic":
                    move[:3, 3] = j.axis * q[k]
                else:
                    move[:3, :3] = axis_angle_to_matrix(j.axis, q[k])
                T = T @ move                      # 관절 변수가 만드는 변환
                k += 1
        return T @ self.tool, np.array(axes), np.array(origins)

    def fk(self, q):
        """⁰T_tip · T_tool (4x4). tool 을 안 줬으면 그냥 ⁰T_tip."""
        return self.frames(q)[0]

    @property
    def has_tool(self):
        return not np.allclose(self.tool, np.eye(4))

    def jacobian(self, q):
        """기하학적 Jacobian (6 x dof). 위 3행이 선속도, 아래 3행이 각속도.

        기준점은 tool 을 줬으면 TCP, 안 줬으면 tip 이다.
        """
        T, axes, origins = self.frames(q)
        p_e = T[:3, 3]
        J = np.zeros((6, self.dof))
        for i, j in enumerate(self.movable):
            z = axes[i]
            if j.type == "prismatic":
                J[:3, i] = z
            else:
                J[:3, i] = np.cross(z, p_e - origins[i])
                J[3:, i] = z
        return J


def metrics(Jv):
    """선속도 Jacobian (3 x n) 에서 강의가 쓰는 지표 세 개를 뽑는다.

    반환 (U, sigma, w, cond)
      sigma — 특이값 σ₁ ≥ σ₂ ≥ σ₃. 타원체의 반축 길이다
      w     — manipulability √(det J Jᵀ) = σ₁σ₂σ₃
      cond  — 조건수 σ₁/σ₃. 특이점에서 ∞ 로 간다
    """
    U, sigma, _ = np.linalg.svd(Jv)
    # det(U) = -1 이면 왼손 좌표계라 쿼터니언이 뒤집힌다. 열 하나를 뒤집어 바로잡는다.
    if np.linalg.det(U) < 0.0:
        U = U.copy()
        U[:, -1] *= -1.0
    w = float(np.prod(sigma))
    cond = float(sigma[0] / sigma[-1]) if sigma[-1] > 1e-12 else float("inf")
    return U, sigma, w, cond
