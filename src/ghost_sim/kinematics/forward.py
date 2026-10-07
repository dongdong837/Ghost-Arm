"""Standard DH forward kinematics; distances in metres, angles in radians."""
import json
from pathlib import Path
import numpy as np
from ghost_sim.paths import CONFIG


def load_parameters(path=CONFIG / 'arm_dh.json'):
    """Read the explicit DH convention and fixed base/tool transforms."""
    data = json.loads(Path(path).read_text())
    if data['convention'] != 'standard_DH_Rz_Tz_Tx_Rx':
        raise ValueError('Only standard DH is supported.')
    return data


def dh_matrix(a, alpha, d, theta):
    """A = Rz(theta) Tz(d) Tx(a) Rx(alpha)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def forward_kinematics(positions, parameters=None):
    """Return base_link -> grasp_center, not a world-frame pose."""
    cfg = load_parameters() if parameters is None else parameters
    q = np.asarray(positions, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError('Expected four finite joint angles in radians.')
    transform = np.eye(4)
    transform[:3, 3] = cfg['base_translation_m']
    for angle, row in zip(q, cfg['rows']):
        transform = transform @ dh_matrix(
            row['a_m'], row['alpha_rad'], row['d_m'],
            angle + row['theta_offset_rad'])
    tool = np.eye(4)
    tool[:3, :3] = cfg['tip_rotation']
    return transform @ tool
