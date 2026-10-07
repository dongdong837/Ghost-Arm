"""Analytical position + pitch IK for the existing yaw/shoulder/elbow/wrist arm."""
import itertools
import math
import numpy as np
from ghost_sim.kinematics.forward import forward_kinematics, load_parameters
from ghost_sim.simulation.tail_structure import PARTS

ARM_NAMES = ('tail_yaw', 'tail_shoulder', 'tail_elbow', 'tail_wrist')
LIMITS = np.array([next(part[6] for part in PARTS if part[2] == name) for name in ARM_NAMES])


def finite_vector(value, size, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{label} 必须包含 {size} 个有限数值。')
    return result


def equivalents(angle, lower, upper):
    """All 2*pi-equivalent angles inside a bounded physical joint interval."""
    first = math.ceil((lower - angle - 1e-10) / math.tau)
    last = math.floor((upper - angle + 1e-10) / math.tau)
    return [float(np.clip(angle + k * math.tau, lower, upper)) for k in range(first, last + 1)]


def inverse_kinematics(position, pitch=0.0, seed=None):
    """Position is in base_link; pitch is q2+q3+q4 modulo 2*pi.

    Yaw follows target direction, not an independently specified orientation.
    Return all isolated valid solutions and the one nearest seed. On the yaw
    axis, choose seed yaw as a representative of the infinitely many yaws.
    This solver does not check collisions or command motion.
    """
    target = finite_vector(position, 3, '目标位置')
    reference = finite_vector([0, 0, 0, 0] if seed is None else seed, 4, '参考关节角')
    if np.any(reference < LIMITS[:, 0] - 1e-10) or np.any(reference > LIMITS[:, 1] + 1e-10):
        raise ValueError('参考关节角超出机械限位。')
    if not np.isfinite(pitch):
        raise ValueError('俯仰角必须为有限弧度值。')
    phi = math.atan2(math.sin(pitch), math.cos(pitch))
    cfg = load_parameters()
    rows = cfg['rows']
    if len(rows) != 4 or tuple(row['joint'] for row in rows) != ARM_NAMES:
        raise ValueError('DH 表必须按尾根、肩、肘、腕的顺序定义四个关节。')
    # The analytical derivation is specific to this topology, not arbitrary DH.
    expected = [(0, -math.pi/2, rows[0]['d_m'], 0),
                (rows[1]['a_m'], 0, 0, math.pi/2),
                (rows[2]['a_m'], 0, 0, 0), (rows[3]['a_m'], 0, 0, 0)]
    actual = [(r['a_m'], r['alpha_rad'], r['d_m'], r['theta_offset_rad']) for r in rows]
    if not np.allclose(actual, expected, atol=1e-12, rtol=0):
        raise ValueError('DH 结构已改变，此解析逆解只适用于当前四轴尾臂。')
    if not np.allclose(cfg['tip_rotation'], [[0,0,-1],[-1,0,0],[0,1,0]], atol=1e-12):
        raise ValueError('工具坐标系已改变，需要重新推导逆解。')
    l1, l2, tool = [r['a_m'] for r in rows[1:]]
    if min(l1, l2, tool) <= 0:
        raise ValueError('连杆长度必须为正数。')
    base = np.asarray(cfg['base_translation_m'], dtype=float)
    dx, dy = target[:2] - base[:2]
    radius = math.hypot(dx, dy)
    on_axis = radius < 1e-10
    headings = [reference[0]] if on_axis else [math.atan2(dy, dx), math.atan2(dy, dx) + math.pi]
    candidates = []
    geometric_reachable = False
    for heading in headings:
        # Signed radius handles targets behind the shoulder plane direction.
        signed_radius = dx * math.cos(heading) + dy * math.sin(heading)
        horizontal = -signed_radius - tool * math.sin(phi)
        downward = -(target[2] - base[2] - rows[0]['d_m']) - tool * math.cos(phi)
        cosine = (horizontal**2 + downward**2 - l1*l1 - l2*l2) / (2*l1*l2)
        if abs(cosine) > 1 + 1e-10:
            continue
        geometric_reachable = True
        elbow = math.acos(float(np.clip(cosine, -1, 1)))
        for q3 in (elbow, -elbow):
            q2 = math.atan2(horizontal, downward) - math.atan2(l2*math.sin(q3), l1+l2*math.cos(q3))
            q4 = phi - q2 - q3
            options = [equivalents(q, *bound) for q, bound in zip((heading, q2, q3, q4), LIMITS)]
            for q in itertools.product(*options):
                q = np.asarray(q)
                result = forward_kinematics(q, cfg)
                error = float(np.linalg.norm(result[:3, 3] - target))
                pitch_error = abs(math.atan2(math.sin(sum(q[1:])-phi), math.cos(sum(q[1:])-phi)))
                if error > 1e-8 or pitch_error > 1e-8:
                    continue
                if any(np.linalg.norm(q-np.asarray(c['joint_angles_rad'])) < 1e-8 for c in candidates):
                    continue
                candidates.append({'joint_angles_rad': q.tolist(), 'position_error_m': error,
                                   'pitch_error_rad': pitch_error, 'seed_distance_rad': float(np.linalg.norm(q-reference)),
                                   'elbow_branch': 'positive' if q[2] >= 0 else 'negative',
                                   'near_singular': on_axis or abs(math.sin(q[2])) < 1e-6})
    if not candidates:
        if not geometric_reachable:
            raise ValueError('目标位置与俯仰角组合超出连杆几何可达范围。')
        raise ValueError('存在几何逆解，但所有解均超出关节限位。')
    candidates.sort(key=lambda candidate: candidate['seed_distance_rad'])
    return {'parent_frame': 'base_link', 'child_frame': 'grasp_center',
            'target_position_m': target.tolist(), 'target_pitch_rad': phi,
            'seed_joint_angles_rad': reference.tolist(), 'solution_count': len(candidates),
            'yaw_underdetermined': on_axis, 'selected': candidates[0], 'solutions': candidates,
            'collision_checked': False, 'motion_executed': False}
