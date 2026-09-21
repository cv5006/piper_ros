"""URDF -> FK · 기하학적 Jacobian. numpy 만 쓴다.

강의의 두 수식이 그대로 코드다.

    T₀ₙ = T₁T₂⋯Tₙ        -> Chain.fk()
    Jᵢ = [ zᵢ × (pₑ − pᵢ) ; zᵢ ]  -> Chain.jacobian()

zᵢ · pᵢ 는 ⁰Tᵢ 에서 꺼내는 값이라 운동은 형상의 부산물이다.
MoveIt 과 같은 값인지는 jacobian_check 가 대조한다 (차이 0).
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
        """tool = tip 프레임에서 잰 TCP 오프셋 (3-벡터 또는 4x4).

        기준점을 옮기면 Jacobian 도 바뀐다 — 선속도 열이 zi x (pe - pi) 이고 pe 가 그 점이다.
        """
        root = ET.parse(urdf_path).getroot()
        joints = [Joint(j) for j in root.findall("joint")]
        by_child = {j.child: j for j in joints}

        # tip 에서 base 까지 부모를 거슬러 올라간 뒤 뒤집는다.
        chain, link = [], tip
        while link != base:
            if link not in by_child:
                raise ValueError(f"no parent joint for '{link}' in the URDF; check base='{base}'")
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
                raise ValueError("tool must be a 3-vector or a 4x4 matrix")
        self.joints = chain
        self.movable = [j for j in chain if j.movable]
        self.names = [j.name for j in self.movable]
        self.lower = np.array([j.lower for j in self.movable])
        self.upper = np.array([j.upper for j in self.movable])

    @property
    def dof(self):
        return len(self.movable)

    def frames(self, q):
        """q -> (T_tip, axes, origins).

          T_tip   ⁰T_tip · T_tool (4x4)
          axes    가동관절 축의 월드 방향 zᵢ
          origins 가동관절 원점의 월드 위치 pᵢ
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
        """기하학적 Jacobian (6 x dof). 위 3행 선속도, 아래 3행 각속도.

        기준점은 tool 이 있으면 TCP, 없으면 tip.
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
    """선속도 Jacobian (3 x n) -> (U, sigma, w, cond).

      sigma  특이값 σ₁ ≥ σ₂ ≥ σ₃ = 타원체 반축
      w      manipulability √(det J Jᵀ) = σ₁σ₂σ₃
      cond   σ₁/σ₃. 특이점에서 ∞
    """
    U, sigma, _ = np.linalg.svd(Jv)
    # det(U) = -1 이면 왼손 좌표계라 쿼터니언이 뒤집힌다. 열 하나를 뒤집어 바로잡는다.
    if np.linalg.det(U) < 0.0:
        U = U.copy()
        U[:, -1] *= -1.0
    w = float(np.prod(sigma))
    cond = float(sigma[0] / sigma[-1]) if sigma[-1] > 1e-12 else float("inf")
    return U, sigma, w, cond


class Visuals:
    """링크를 어디에 어떤 메시로 그리는지. 로봇을 여러 벌 그릴 때 쓴다.

    MoveIt 의 Trajectory 디스플레이가 SRDF 를 요구해 URDF 스택에서 뜨지 못한다.
    링크 메시를 Marker 로 직접 놓으면 rviz 기본 플러그인만으로 된다.

    Chain 과 달리 사슬 하나가 아니라 **트리 전체**를 본다 (손가락 같은 가지 포함).
    """

    def __init__(self, urdf_path, base="base_link"):
        root = ET.parse(urdf_path).getroot()
        self.base = base

        self.mesh = {}          # 링크 -> (메시 URI, 링크 프레임에서의 4x4, 배율 3-벡터)
        for link in root.findall("link"):
            mesh = link.find("visual/geometry/mesh")
            if mesh is None:                       # 메시가 없는 링크는 그리지 않는다
                continue
            origin = link.find("visual/origin")
            T = np.eye(4)
            if origin is not None:
                if origin.get("rpy"):
                    T[:3, :3] = rpy_to_matrix([float(v) for v in origin.get("rpy").split()])
                if origin.get("xyz"):
                    T[:3, 3] = [float(v) for v in origin.get("xyz").split()]
            scale = [float(v) for v in (mesh.get("scale") or "1 1 1").split()]
            self.mesh[link.get("name")] = (mesh.get("filename"), T, scale)

        self.children = {}
        for j in (Joint(e) for e in root.findall("joint")):
            self.children.setdefault(j.parent, []).append(j)

    @property
    def links(self):
        """그릴 수 있는 링크를 URDF 에 적힌 순서로."""
        return list(self.mesh)

    def link_poses(self, q_by_name):
        """관절값(이름 -> 값, 빠진 것은 0) -> 모든 링크의 ⁰T.

        base 에서 트리를 내려가며 T_child = T_parent · origin · move 를 쌓는다.
        """
        poses = {self.base: np.eye(4)}
        stack = [self.base]
        while stack:
            parent = stack.pop()
            for j in self.children.get(parent, []):
                T = poses[parent] @ j.origin
                if j.movable:
                    v = float(q_by_name.get(j.name, 0.0))
                    move = np.eye(4)
                    if j.type == "prismatic":
                        move[:3, 3] = j.axis * v
                    else:
                        move[:3, :3] = axis_angle_to_matrix(j.axis, v)
                    T = T @ move
                poses[j.child] = T
                stack.append(j.child)
        return poses
