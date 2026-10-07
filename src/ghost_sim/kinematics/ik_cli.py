"""Analytical inverse kinematics with optional execution; position in base_link, pitch in radians."""
import argparse
import json
from ghost_sim.kinematics.inverse import inverse_kinematics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('position', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--pitch', type=float, default=0.0,
                        help='q2+q3+q4; 0 keeps the gripper pointing down relative to the body')
    seeds = parser.add_mutually_exclusive_group()
    seeds.add_argument('--seed', nargs=4, type=float, metavar=('Q1', 'Q2', 'Q3', 'Q4'))
    seeds.add_argument('--live', action='store_true', help='read current Gazebo joints as the reference; do not move')
    parser.add_argument('--execute', action='store_true', help='solve from current joints and move the arm; fingers stay unchanged')
    parser.add_argument('--duration', type=float, default=4.0, help='minimum motion duration in simulation seconds')
    args = parser.parse_args()
    if args.execute and args.seed is not None:
        parser.error('--execute uses actual joint feedback; do not pass --seed')
    try:
        if args.execute:
            from ghost_sim.control.ik_execute import execute_target
            result = execute_target(args.position, args.pitch, args.duration)
        else:
            seed = args.seed
            if args.live:
                from ghost_sim.kinematics.fk_cli import live_sample
                seed, _, _ = live_sample()
            result = inverse_kinematics(args.position, args.pitch, seed)
    except (ValueError, RuntimeError) as error:
        print(json.dumps({'success': False, 'error': str(error), 'motion_executed': getattr(error, 'motion_started', False)}, ensure_ascii=False, indent=2))
        raise SystemExit(1)
    print(json.dumps({'success': True, **result}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
