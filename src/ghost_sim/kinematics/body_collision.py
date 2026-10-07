"""Arm box versus body sphere checks in base_link, including sampled joint paths.

Checks the model's collision geometry, not room obstacles or arm-arm collisions.
The fixed mounting parts intentionally join the body and are excluded.
"""
import math
import numpy as np
from ghost_sim.simulation.tail_structure import PARTS

BODY_RADIUS = .28
MARGIN = .005


def arm_body_clearance(joints):
    q = np.asarray(joints, dtype=float)
    if q.shape not in ((4,), (6,)) or not np.isfinite(q).all():
        raise ValueError('碰撞检查需要 4 或 6 个有限关节值。')
    values = dict(zip(('tail_yaw', 'tail_shoulder', 'tail_elbow', 'tail_wrist',
                       'finger_left_slide', 'finger_right_slide'), q))
    frames = {'base_link': np.eye(4)}
    distances = {}
    for parent, child, name, kind, origin, axis, limits, size, centre, mass in PARTS:
        local = np.eye(4)
        local[:3, 3] = origin
        if kind == 'revolute':
            a = np.asarray(axis, dtype=float)
            cross = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            angle = values[name]
            local[:3, :3] = np.eye(3)+math.sin(angle)*cross+(1-math.cos(angle))*(cross@cross)
        elif kind == 'prismatic':
            local[:3, 3] += np.asarray(axis)*values.get(name, 0.)
        frame = frames[parent] @ local
        frames[child] = frame
        if size is None or kind == 'fixed':
            continue
        # Transform the sphere centre into the oriented link box's coordinates.
        sphere_in_box = -frame[:3, :3].T @ frame[:3, 3]-np.asarray(centre)
        closest = np.clip(sphere_in_box, -np.asarray(size)/2, np.asarray(size)/2)
        distances[child] = float(np.linalg.norm(sphere_in_box-closest)-BODY_RADIUS)
    link = min(distances, key=distances.get)
    return distances[link], link


def check_body_path(start, target, step=.01):
    """Sample the straight joint-space path (smoothstep follows this same curve).

    This is a bounded-resolution check, not a continuous collision proof.
    """
    start, target = np.asarray(start, dtype=float), np.asarray(target, dtype=float)
    if start.shape != target.shape or start.shape not in ((4,), (6,)):
        raise ValueError('起点和终点必须有相同的 4 或 6 个关节。')
    if not np.isfinite(start).all() or not np.isfinite(target).all():
        raise ValueError('关节值必须有限。')
    count = max(1, math.ceil(float(np.max(np.abs(target-start)))/step))
    minimum, closest = float('inf'), ''
    for t in np.linspace(0., 1., count+1):
        distance, link = arm_body_clearance(start+t*(target-start))
        if distance < minimum:
            minimum, closest = distance, link
        if distance < MARGIN:
            raise ValueError(f'机械臂路径靠近或穿过球体：{link}，间隙 {distance*1000:.1f} mm，要求至少 {MARGIN*1000:.0f} mm。')
    return {'scope': 'arm_boxes_vs_body_sphere', 'min_clearance_m': minimum,
            'closest_link': closest, 'max_joint_sample_step': step, 'samples': count+1}


def select_body_safe_solution(result, start):
    """Filter analytical IK branches by the entire execution path, retaining seed order."""
    reasons = []
    for candidate in result['solutions']:
        try:
            check = check_body_path(start, candidate['joint_angles_rad'])
        except ValueError as error:
            reasons.append(str(error))
            continue
        result['selected'] = candidate
        result['body_collision_check'] = check
        return result
    raise ValueError('没有避开球体的直接关节路径。'+(' '+reasons[0] if reasons else ''))
