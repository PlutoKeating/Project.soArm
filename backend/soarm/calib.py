"""深度相机到机械臂基座的外参标定（触点法）。

做法：把夹爪尖移动到深度相机能看见的若干位置，每个位置在深度画面上点一下夹爪尖。
这样同一个物理点就有两组坐标：相机系下的（来自深度反投影）和基座系下的（来自正运动学）。
凑够不共面的 4 组以上，用 Kabsch 算法解出刚体变换 T_base_cam。

外参只在云台处于标定时的姿态下有效，所以结果里同时记录了云台预设角。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

MIN_SAMPLES = 4


def solve_rigid(cam_pts: np.ndarray, base_pts: np.ndarray) -> tuple[np.ndarray, float]:
    """求 T 使 base ≈ T @ cam。返回 (4x4 变换, RMS 残差/米)。"""
    cam_pts, base_pts = np.asarray(cam_pts, float), np.asarray(base_pts, float)
    if len(cam_pts) < MIN_SAMPLES:
        raise ValueError(f"至少需要 {MIN_SAMPLES} 个采样点")
    cc, bc = cam_pts.mean(axis=0), base_pts.mean(axis=0)
    H = (cam_pts - cc).T @ (base_pts - bc)
    if np.linalg.matrix_rank(H) < 3:
        raise ValueError("采样点共面或共线，请在不同高度和远近多采几个点")
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = bc - R @ cc
    residual = base_pts - (cam_pts @ R.T + T[:3, 3])
    return T, float(np.sqrt((residual**2).sum(axis=1).mean()))


@dataclass
class BearingSample:
    """同一只手的一次配对观测：臂上腕部相机看到它的视线 + 深度相机测到它的 3D 位置。"""

    origin: np.ndarray  # 视线起点（腕部相机位置），臂基座系，米
    direction: np.ndarray  # 视线单位方向，臂基座系
    cam_point: np.ndarray  # 手在深度相机光学系下的位置，米


# 深度相机光学系 (x 右, y 下, z 前) 到臂基座系 (x 前, y 左, z 上) 的名义朝向
_NOMINAL_R = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
MIN_BEARING_SAMPLES = 10


def _rotvec_to_matrix(w: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(w))
    if angle < 1e-12:
        return np.eye(3)
    k = w / angle
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def _bearing_residuals(params, R0, origins, dirs, pts):
    """每个样本两到三个分量：手的位置偏离视线的角度（弧度）。"""
    R = R0 @ _rotvec_to_matrix(params[:3])
    v = pts @ R.T + params[3:] - origins
    along = np.einsum("ij,ij->i", v, dirs)
    perp = v - along[:, None] * dirs
    return (perp / np.maximum(np.linalg.norm(v, axis=1), 0.05)[:, None]).ravel()


def _gauss_newton(R0, t0, origins, dirs, pts, iters: int = 40):
    params = np.concatenate([np.zeros(3), t0])
    lam = 1e-3
    res = _bearing_residuals(params, R0, origins, dirs, pts)
    cost = float(res @ res)
    for _ in range(iters):
        J = np.empty((res.size, 6))
        for i in range(6):
            step = np.zeros(6)
            step[i] = 1e-5
            J[:, i] = (_bearing_residuals(params + step, R0, origins, dirs, pts) - res) / 1e-5
        delta = np.linalg.solve(J.T @ J + lam * np.eye(6), -J.T @ res)
        trial = params + delta
        trial_res = _bearing_residuals(trial, R0, origins, dirs, pts)
        trial_cost = float(trial_res @ trial_res)
        if trial_cost < cost:
            params, res, cost, lam = trial, trial_res, trial_cost, max(lam / 3, 1e-9)
            if float(np.linalg.norm(delta)) < 1e-9:
                break
        else:
            lam *= 5
    return R0 @ _rotvec_to_matrix(params[:3]), params[3:], cost


def solve_from_bearings(samples: list[BearingSample]) -> tuple[np.ndarray, float, int]:
    """由“视线 + 3D 点”配对求 T_base_cam。返回 (4x4 变换, 手偏离视线的 RMS 距离/米, 采用的样本数)。

    这是一个 PnP 问题：深度相机给出手的 3D 位置，臂上的腕部相机给出它的方向。
    从多个初始朝向出发做非线性最小二乘，取最好的；再剔除明显的离群样本重解一次
    （两台相机偶尔看到的不是同一只手）。
    """
    if len(samples) < MIN_BEARING_SAMPLES:
        raise ValueError(f"至少需要 {MIN_BEARING_SAMPLES} 个配对样本，当前 {len(samples)} 个")
    origins = np.array([s.origin for s in samples], float)
    dirs = np.array([s.direction for s in samples], float)
    pts = np.array([s.cam_point for s in samples], float)
    if np.linalg.matrix_rank(pts - pts.mean(axis=0), tol=0.02) < 3:
        raise ValueError("样本里手的位置几乎都在一个平面或一条线上，请在不同远近、高低、左右多移动一些")

    def best_fit(o, d, p):
        best = None
        for pitch in (0.0, 25.0, 50.0, 75.0):  # 相机低头的角度
            for yaw in (-40.0, 0.0, 40.0):
                cp, sp = np.cos(np.deg2rad(pitch)), np.sin(np.deg2rad(pitch))
                cy, sy = np.cos(np.deg2rad(yaw)), np.sin(np.deg2rad(yaw))
                Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
                Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
                R0 = Rz @ Ry @ _NOMINAL_R
                # 初始平移：让点云中心落在视线上离起点 0.35 米处
                t0 = (o + 0.35 * d).mean(axis=0) - (p @ R0.T).mean(axis=0)
                R, t, cost = _gauss_newton(R0, t0, o, d, p)
                if best is None or cost < best[2]:
                    best = (R, t, cost)
        return best

    def distances(R, t, o, d, p):
        v = p @ R.T + t - o
        return np.linalg.norm(v - np.einsum("ij,ij->i", v, d)[:, None] * d, axis=1)

    R, t, _ = best_fit(origins, dirs, pts)
    dist = distances(R, t, origins, dirs, pts)
    keep = dist <= max(3.0 * float(np.median(dist)), 0.02)
    if MIN_BEARING_SAMPLES <= int(keep.sum()) < len(samples):
        origins, dirs, pts = origins[keep], dirs[keep], pts[keep]
        R, t, _ = best_fit(origins, dirs, pts)
        dist = distances(R, t, origins, dirs, pts)
    # 手必须在腕部相机前方，否则是镜像解
    if float(np.mean(np.einsum("ij,ij->i", pts @ R.T + t - origins, dirs))) <= 0:
        raise ValueError("求解结果不合理（手落在相机后方），请重新采集")
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T, float(np.sqrt((dist**2).mean())), len(pts)


def refine_from_bearings(
    samples: list[BearingSample],
    T_init: np.ndarray,
    up_cam: np.ndarray,
    gate_deg: float = 25.0,
) -> tuple[np.ndarray, float, int]:
    """用“视线 + 3D 点”配对在已有外参的基础上精修。返回 (4x4 变换, 手偏离视线的 RMS 角度/度, 采用的样本数)。

    与从零求解不同：俯仰和横滚仍由地面法线 up_cam 定死，只调绕竖直轴的转角和平移；
    并且先用现有外参筛一遍，手的位置偏离腕部视线超过 gate_deg 的配对直接不要
    （展会上两台相机经常看到的不是同一只手）。
    """
    origins = np.array([s.origin for s in samples], float)
    dirs = np.array([s.direction for s in samples], float)
    pts = np.array([s.cam_point for s in samples], float)

    def angles(R, t):
        v = pts @ R.T + t - origins
        cosang = np.einsum("ij,ij->i", v, dirs) / np.maximum(np.linalg.norm(v, axis=1), 1e-6)
        return np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0)))

    keep = angles(T_init[:3, :3], T_init[:3, 3]) <= gate_deg
    if int(keep.sum()) < MIN_BEARING_SAMPLES:
        raise ValueError(f"与现有外参相符的配对只有 {int(keep.sum())} 个，至少需要 {MIN_BEARING_SAMPLES} 个")
    o, d, p = origins[keep], dirs[keep], pts[keep]
    if np.linalg.matrix_rank(p - p.mean(axis=0), tol=0.02) < 3:
        raise ValueError("样本里手的位置几乎都在一个平面或一条线上，请在不同远近、高低、左右多移动一些")

    def residuals(params):
        R = _rotation_from_up(up_cam, params[0])
        v = p @ R.T + params[1:] - o
        along = np.einsum("ij,ij->i", v, d)
        perp = v - along[:, None] * d
        return (perp / np.maximum(np.linalg.norm(v, axis=1), 0.05)[:, None]).ravel()

    params = np.array([np.arctan2(T_init[1, 2], T_init[0, 2]), *T_init[:3, 3]])
    res = residuals(params)
    cost, lam = float(res @ res), 1e-3
    for _ in range(60):
        J = np.empty((res.size, 4))
        for i in range(4):
            step = np.zeros(4)
            step[i] = 1e-5
            J[:, i] = (residuals(params + step) - res) / 1e-5
        delta = np.linalg.solve(J.T @ J + lam * np.eye(4), -J.T @ res)
        trial = residuals(params + delta)
        if float(trial @ trial) < cost:
            params, res, cost, lam = params + delta, trial, float(trial @ trial), max(lam / 3, 1e-9)
            if float(np.linalg.norm(delta)) < 1e-9:
                break
        else:
            lam *= 5
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = _rotation_from_up(up_cam, params[0]), params[1:]
    # 精修不应该把相机挪走很远：挪得多说明样本有问题，不采用
    if np.linalg.norm(T[:3, 3] - T_init[:3, 3]) > 0.15:
        raise ValueError(
            f"精修后相机位置移动了 {np.linalg.norm(T[:3, 3] - T_init[:3, 3]) * 100:.0f} 厘米，超出合理范围，不采用"
        )
    v = p @ T[:3, :3].T + T[:3, 3] - o
    cosang = np.einsum("ij,ij->i", v, d) / np.maximum(np.linalg.norm(v, axis=1), 1e-6)
    rms = float(np.sqrt((np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0))) ** 2).mean()))
    return T, rms, int(keep.sum())


def estimate_wrist_offset(
    samples: list[BearingSample],
    T_base_cam: np.ndarray,
    initial: tuple[float, float] = (0.0, 0.0),
    gate_deg: float = 22.0,
) -> tuple[tuple[float, float], int, tuple[float, float]]:
    """由配对样本估计腕部相机光轴相对夹爪指向的安装偏角。

    样本里的视线是按“光轴与夹爪指向重合”算的；深度相机测到的手的方向相对它的偏差，
    在各个样本里应当是同一个固定角度，这就是安装偏角。返回
    ((偏右, 偏上), 采用的样本数, (两个方向的散布/度))。

    展会上很多配对不是同一只手，所以用迭代的中位数：只在当前估计附近 gate_deg 以内的
    样本里取中位数，从 initial 出发收敛。
    """
    origins = np.array([s.origin for s in samples], float)
    dirs = np.array([s.direction for s in samples], float)
    pts = np.array([s.cam_point for s in samples], float)
    if len(samples) < MIN_BEARING_SAMPLES:
        raise ValueError(f"至少需要 {MIN_BEARING_SAMPLES} 个配对样本，当前 {len(samples)} 个")
    v = pts @ T_base_cam[:3, :3].T + T_base_cam[:3, 3] - origins
    v /= np.linalg.norm(v, axis=1)[:, None]
    world_up = np.array([0.0, 0.0, 1.0])
    up = world_up - (dirs @ world_up)[:, None] * dirs
    up /= np.linalg.norm(up, axis=1)[:, None]
    right = np.cross(dirs, up)
    along = np.einsum("ij,ij->i", v, dirs)
    e_right = np.degrees(np.arctan2(np.einsum("ij,ij->i", v, right), along))
    e_up = np.degrees(np.arctan2(np.einsum("ij,ij->i", v, up), along))
    estimate = np.array(initial, float)
    near = np.zeros(len(samples), bool)
    for _ in range(8):
        near = np.hypot(e_right - estimate[0], e_up - estimate[1]) < gate_deg
        if int(near.sum()) < MIN_BEARING_SAMPLES:
            raise ValueError(f"相互一致的配对只有 {int(near.sum())} 个，至少需要 {MIN_BEARING_SAMPLES} 个")
        updated = np.array([np.median(e_right[near]), np.median(e_up[near])])
        if np.allclose(updated, estimate, atol=0.05):
            break
        estimate = updated
    spread = (float(e_right[near].std()), float(e_up[near].std()))
    if max(spread) > 9.0:
        raise ValueError(f"样本散布太大（{spread[0]:.0f}° / {spread[1]:.0f}°），估计不可信")
    return (float(estimate[0]), float(estimate[1])), int(near.sum()), spread


@dataclass
class BodyObservation:
    """机械臂摆在某个姿态时，深度相机看到的属于手臂的点。"""

    skeleton: np.ndarray  # 该姿态下的骨架点，臂基座系，(K, 3)
    cam_points: np.ndarray  # 手臂表面的点，相机光学系，(N, 3)


def _segment_distance(points: np.ndarray, skeleton: np.ndarray) -> np.ndarray:
    """每个点到骨架折线的最近距离。"""
    best = np.full(len(points), np.inf)
    for a, b in zip(skeleton[:-1], skeleton[1:]):
        ab = b - a
        length2 = float(ab @ ab)
        if length2 < 1e-10:
            continue
        t = np.clip((points - a) @ ab / length2, 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(points - (a + t[:, None] * ab), axis=1))
    return best


def fit_floor_normal(
    depth: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    expected_up: np.ndarray | None = None,
    max_deviation_deg: float = 12.0,
) -> tuple[np.ndarray, float]:
    """从一帧深度图里找出地面，返回 (相机系下朝上的单位法线, 相机离地高度/米)。

    地面的法线就是世界的“上”，由此可以直接定出相机的俯仰和横滚。相机低头较多时地面是
    画面里最大的平面；相机抬得较平时，正前方的墙面可能更大，这时要给出 expected_up
    （大致的“上”方向，比如由云台转角推算），只接受法线与它相差不大的平面。
    """
    fx, fy, cx, cy = intrinsics
    h, w = depth.shape
    v, u = np.mgrid[0:h:6, 0:w:6]
    z = depth[v, u]
    ok = (z > 0.6) & (z < 4.0)  # 0.6 米以内是手臂和近处的人，不是地面
    if ok.sum() < 200:
        raise ValueError("画面里几乎没有地面，无法确定相机的俯仰")
    pts = np.stack([(u[ok] - cx) / fx * z[ok], (v[ok] - cy) / fy * z[ok], z[ok]], axis=1)
    rng = np.random.default_rng(0)
    best_count, best_mask = 0, None
    for _ in range(600):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-6:
            continue
        n = n / norm
        if expected_up is not None and abs(float(n @ expected_up)) < np.cos(np.deg2rad(max_deviation_deg)):
            continue
        mask = np.abs((pts - a) @ n) < 0.02
        if mask.sum() > best_count:
            best_count, best_mask = int(mask.sum()), mask
    if best_mask is None or best_count < 150:
        raise ValueError("没有找到方向相符的地面")
    plane = pts[best_mask]
    centre = plane.mean(axis=0)
    normal = np.linalg.svd(plane - centre)[2][-1]
    if expected_up is not None:
        # 有先验时按先验定正反，并要求这个平面在相机下方。否则相机上方一块朝下的平面
        # （比如有人举着的板子）也会被当成地面，而且“上”的方向正好反过来
        if normal @ expected_up < 0:
            normal = -normal
        if normal @ centre > 0:
            raise ValueError("方向相符的平面在相机上方，不是地面")
    elif normal @ centre > 0:
        normal = -normal  # 朝向相机一侧，即朝上
    return normal, float(abs(normal @ centre))


def _rotation_from_up(up_cam: np.ndarray, yaw: float) -> np.ndarray:
    """相机系到基座系的旋转：up_cam 映到基座 +z，光轴的水平分量指向方位角 yaw。"""
    z_axis = up_cam / np.linalg.norm(up_cam)
    forward = np.array([0.0, 0.0, 1.0]) - z_axis[2] * z_axis  # 光轴在水平面内的投影（相机系下）
    if np.linalg.norm(forward) < 1e-6:
        forward = np.array([0.0, -1.0, 0.0]) - (-z_axis[1]) * z_axis  # 相机垂直朝下时用画面的“上”
    x_axis = forward / np.linalg.norm(forward)
    y_axis = np.cross(z_axis, x_axis)
    R_level = np.stack([x_axis, y_axis, z_axis])  # 相机系 -> “相机朝前的水平系”
    c, s_ = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]]) @ R_level


def solve_from_body(
    observations: list[BodyObservation],
    up_cam: np.ndarray,
    link_radius: float = 0.025,
    yaw_deg: float | None = None,
    all_candidates: bool = False,
):
    """用机械臂自身当标定物求 T_base_cam。返回 (4x4 变换, 点到手臂表面的 RMS 距离/米, 采用的点数)。

    all_candidates 为真时返回所有互不相同的候选解的列表（按残差从小到大），由调用方
    再用 verify_body 的反向验证来挑：残差最小的那个不一定对。

    让机械臂摆若干姿态，深度相机每次看到的手臂点云，换到基座系后都应该贴在该姿态的
    骨架周围。几点做法：
    - 相机的俯仰和横滚不拟合，直接用地面法线 up_cam 定死（假定机械臂底座是水平的）。
      只解绕竖直轴的 1 个转角加 3 个平移，共 4 个参数，条件比解 6 个好得多。
    - 两个方向的约束都要：看到的点要贴着模型；模型的夹爪处也必须有点。只有前者的话，
      模型可以沿手臂方向滑出去而不受罚。
    - 先粗解，把离模型远的点（另一只臂、路过的人）剔掉，再精解。
    - yaw_deg 给定时连这个转角也不解，只解平移。手臂只能在相机视野里的一小片区域出现时，
      “相机绕竖直轴转一点”和“相机平移一点”几乎分不开，硬解会差出几十度；而相机相对机械臂
      的朝向通常是已知的（装在两臂正中、朝向一致就是 0）。
    """
    if len(observations) < 4:
        raise ValueError("至少需要 4 个姿态")
    skeletons = [o.skeleton for o in observations]
    clouds = [np.asarray(o.cam_points, float) for o in observations]
    if sum(len(c) for c in clouds) < 200:
        raise ValueError("手臂点云太少：相机可能看不到手臂")
    cap = 0.08  # 残差封顶：远处的点不让它们主导结果
    # 每个姿态在夹爪那一段上取几个模型点，用来做“模型处必须有点”的反向约束
    probes = [np.array([sk[-2] + f * (sk[-1] - sk[-2]) for f in (0.35, 0.6, 0.85, 1.0)]) for sk in skeletons]

    def transform(params):
        yaw = params[0] if yaw_deg is None else np.deg2rad(yaw_deg)
        return _rotation_from_up(up_cam, yaw), params[1:]

    def residuals(params, use):
        R, t = transform(params)
        out = []
        for skel, cloud, probe, keep in zip(skeletons, clouds, probes, use):
            pts = cloud[keep] @ R.T + t
            out.append(np.clip(_segment_distance(pts, skel) - link_radius, -cap, cap))
            if len(pts):
                nearest = np.linalg.norm(pts[None, :, :] - probe[:, None, :], axis=2).min(axis=1)
                out.append(2.0 * np.clip(nearest - link_radius, 0.0, cap))
        return np.concatenate(out)

    def refine(params, use, iters):
        lam = 1e-2
        res = residuals(params, use)
        cost = float(res @ res)
        steps = np.array([1e-4, 1e-4, 1e-4, 1e-4])
        for _ in range(iters):
            J = np.empty((res.size, 4))
            for i in range(4):
                d = np.zeros(4)
                d[i] = steps[i]
                J[:, i] = (residuals(params + d, use) - res) / steps[i]
            if yaw_deg is not None:
                J[:, 0] = 0.0  # 转角固定：不让它动
            delta = np.linalg.solve(J.T @ J + lam * np.eye(4), -J.T @ res)
            trial_res = residuals(params + delta, use)
            trial_cost = float(trial_res @ trial_res)
            if trial_cost < cost:
                params, res, cost, lam = params + delta, trial_res, trial_cost, max(lam / 3, 1e-8)
                if float(np.linalg.norm(delta)) < 1e-7:
                    break
            else:
                lam *= 4
                if lam > 1e6:
                    break
        return params, cost

    everything = [np.ones(len(c), bool) for c in clouds]
    total = sum(len(c) for c in clouds)
    starts = []
    for yaw in np.deg2rad([-40.0, -20.0, 0.0, 20.0, 40.0] if yaw_deg is None else [yaw_deg]):
        for x in (-0.1, 0.05, 0.2):
            for y in (-0.3, -0.15, 0.0, 0.15, 0.3):
                for z in (0.3, 0.45, 0.6):
                    starts.append(refine(np.array([yaw, x, y, z]), everything, 10))
    starts.sort(key=lambda item: item[1])
    # 不同初值会收敛到几个不同的解（典型的是沿手臂方向错开一节连杆）。把互不相同的都精解出来
    distinct: list[np.ndarray] = []
    for params, _ in starts:
        if all(np.linalg.norm(params[1:] - d_[1:]) > 0.05 for d_ in distinct):
            distinct.append(params)
        if len(distinct) >= 4:
            break
    candidates, errors = [], []
    for params in distinct:
        params, _ = refine(params, everything, 40)
        R, t = transform(params)
        # 剔除离模型远的点后精解
        use = [np.abs(_segment_distance(c @ R.T + t, sk) - link_radius) < 0.06 for sk, c in zip(skeletons, clouds)]
        kept = sum(int(u.sum()) for u in use)
        if kept < max(200, 0.3 * total):
            errors.append(f"只有 {kept / total:.0%} 的点能贴到手臂模型上")
            continue
        params, cost = refine(params, use, 60)
        R, t = transform(params)
        # 合理性：相机装在手臂上方、朝前。对着一堆无关的点也可能凑出一个残差不大的解，在这里拦下
        if not 0.15 < t[2] < 1.0 or R[0, 2] < 0.1:
            errors.append(f"相机位置 ({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}) 或朝向 {np.round(R[:, 2], 2).tolist()} 不合理")
            continue
        dist = np.concatenate([
            (_segment_distance(c @ R.T + t, sk) - link_radius)[u] for sk, c, u in zip(skeletons, clouds, use)
        ])
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, t
        candidates.append((T, float(np.sqrt((dist**2).mean())), kept, cost))
    if not candidates:
        raise ValueError(
            "没有可信的解：" + "；".join(errors or ["未收敛"])
            + "。相机应在手臂上方、朝前；多半是手臂没进入相机视野，点云里是别的东西"
        )
    candidates.sort(key=lambda item: item[3])
    if all_candidates:
        return [(T, rms, kept) for T, rms, kept, _ in candidates]
    T, rms, kept, _ = candidates[0]
    return T, rms, kept


def verify_body(
    T: np.ndarray,
    skeletons: list[np.ndarray],
    depths: list[np.ndarray],
    intrinsics: tuple[float, float, float, float],
    tolerance: float = 0.05,
) -> tuple[float, float]:
    """反向验证外参：把每个姿态的手臂模型投影到深度图上，看深度图是否真的在那附近看到了东西。

    返回 (模型点落在画面内的比例, 其中被深度证实的比例)。求解只保证“看到的点贴着模型”，
    这里检查“模型在的地方确实有东西”，两头都成立才可信。骨架只是连杆的轴线，而夹爪是个
    不对称的宽块，所以允许 tolerance 这么大的偏差：在画面上看周围这么大一圈，深度上也放宽这么多。
    """
    fx, fy, cx, cy = intrinsics
    T_inv = np.linalg.inv(T)
    total = inside = confirmed = 0
    for skel, depth in zip(skeletons, depths):
        h, w = depth.shape
        for a, b in zip(skel[2:-1], skel[3:]):  # 前臂与夹爪：会进入视野的部分
            for f in np.linspace(0.1, 0.9, 12):
                p = T_inv[:3, :3] @ (a + f * (b - a)) + T_inv[:3, 3]
                total += 1
                if p[2] <= 0.05:
                    continue
                u, v = int(round(fx * p[0] / p[2] + cx)), int(round(fy * p[1] / p[2] + cy))
                if not (0 <= u < w and 0 <= v < h):
                    continue
                inside += 1
                r = max(2, int(round(fx * tolerance / p[2])))
                patch = depth[max(0, v - r) : v + r + 1, max(0, u - r) : u + r + 1]
                valid = patch[patch > 0.05]
                if valid.size and np.any(np.abs(valid - p[2]) < tolerance):
                    confirmed += 1
    return inside / max(total, 1), confirmed / max(inside, 1)


def rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """把单位向量 a 转到 b 的最小旋转。"""
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    axis = np.cross(a, b)
    s_, c = float(np.linalg.norm(axis)), float(a @ b)
    if s_ < 1e-9:
        return np.eye(3)
    return _rotvec_to_matrix(axis / s_ * np.arctan2(s_, c))


@dataclass
class ArmExtrinsic:
    T_base_cam: np.ndarray
    rms: float
    samples: int
    # 标定时地面法线在相机系下的方向（朝上）。云台俯仰变了之后，用当前测到的地面法线
    # 与它比较，就能把外参换算到当前俯仰下
    up_cam: np.ndarray | None = None
    # 这只臂腕部相机光轴相对夹爪指向的安装偏角 (偏右, 偏上)，度。由自动样本估计
    wrist_offset: tuple[float, float] = (0.0, 0.0)

    def at(self, up_now: np.ndarray | None) -> np.ndarray:
        """当前俯仰下的 T_base_cam。up_now 是此刻地面法线在相机系下的方向。

        假定云台只是点头（绕相机的左右轴转），没有左右摇头；并忽略俯仰轴与相机光心之间
        几厘米的偏距。
        """
        if up_now is None or self.up_cam is None:
            return self.T_base_cam
        T = self.T_base_cam.copy()
        # 当前相机系里的点先转到标定时的相机系，再用标定时的外参
        T[:3, :3] = self.T_base_cam[:3, :3] @ rotation_between(up_now, self.up_cam)
        return T


@dataclass
class Calibration:
    head_preset: dict[str, float] | None = None  # 标定时的云台角 {"pan", "tilt"}
    arms: dict[str, ArmExtrinsic] = field(default_factory=dict)

    @classmethod
    def load(cls, path: Path) -> "Calibration":
        if not path.exists():
            return cls()
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        arms = {
            name: ArmExtrinsic(
                np.array(a["T_base_cam"], float), float(a["rms"]), int(a["samples"]),
                None if a.get("up_cam") is None else np.array(a["up_cam"], float),
                tuple(float(v) for v in a.get("wrist_offset", (0.0, 0.0))),
            )
            for name, a in (data.get("arms") or {}).items()
        }
        return cls(head_preset=data.get("head_preset"), arms=arms)

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "head_preset": self.head_preset,
            "arms": {
                name: {
                    "T_base_cam": [[round(float(v), 6) for v in row] for row in a.T_base_cam],
                    "rms": round(a.rms, 5),
                    "samples": a.samples,
                    "up_cam": None if a.up_cam is None else [round(float(v), 5) for v in a.up_cam],
                    "wrist_offset": [round(float(v), 2) for v in a.wrist_offset],
                }
                for name, a in self.arms.items()
            },
        }
        path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")


class CalibSession:
    """采样暂存。每只臂各自一组点对。"""

    def __init__(self):
        self.samples: dict[str, list[tuple[np.ndarray, np.ndarray]]] = {}

    def add(self, arm: str, cam_pt: np.ndarray, base_pt: np.ndarray) -> int:
        self.samples.setdefault(arm, []).append((np.asarray(cam_pt, float), np.asarray(base_pt, float)))
        return len(self.samples[arm])

    def undo(self, arm: str) -> int:
        pts = self.samples.get(arm, [])
        if pts:
            pts.pop()
        return len(pts)

    def clear(self, arm: str) -> None:
        self.samples.pop(arm, None)

    def count(self, arm: str) -> int:
        return len(self.samples.get(arm, []))

    def solve(self, arm: str) -> ArmExtrinsic:
        pts = self.samples.get(arm, [])
        cam = np.array([p[0] for p in pts]).reshape(-1, 3)
        base = np.array([p[1] for p in pts]).reshape(-1, 3)
        T, rms = solve_rigid(cam, base)
        return ArmExtrinsic(T, rms, len(pts))
