"""Regression: a valid end-effector IK can place its upper arm inside the body."""
import unittest
import numpy as np
from ghost_sim.kinematics.body_collision import arm_body_clearance, check_body_path, select_body_safe_solution
from ghost_sim.kinematics.inverse import inverse_kinematics


class BodyCollisionTests(unittest.TestCase):
    def test_old_shortest_ik_penetrates_sphere(self):
        answer = inverse_kinematics([.025,0,-.427],0)
        q = answer['selected']['joint_angles_rad']
        distance, link = arm_body_clearance(q)
        self.assertEqual(link, 'tail_upper')
        self.assertLess(distance, -.09)
        with self.assertRaises(ValueError):check_body_path([0]*4, q)

    def test_new_grasp_selects_clear_elbow_branch(self):
        raw = inverse_kinematics([0,0,-.46],0)
        self.assertGreater(raw['selected']['joint_angles_rad'][2],0)
        answer = select_body_safe_solution(raw,[0]*4)
        self.assertLess(answer['selected']['joint_angles_rad'][2],0)
        self.assertGreater(answer['body_collision_check']['min_clearance_m'], .02)
        q = answer['selected']['joint_angles_rad']
        check_body_path(np.r_[q,.025,.025],np.r_[q,0,0])

    def test_clear_endpoints_can_have_colliding_middle(self):
        # This path swings the arm through the body although both ends are clear.
        start = [0.37994219995,0.08398376956,-1.53458441798,-2.06479252568]
        target = [-0.45574869858,0.18041562276,-1.72224486841,-0.07629484973]
        for q in (start, target):
            self.assertGreater(arm_body_clearance(q)[0],.005)
        with self.assertRaises(ValueError):check_body_path(start,target)

    def test_sixty_degree_grasp_path_and_floor_clearance(self):
        import math
        from ghost_sim.kinematics.forward import forward_kinematics
        pitch=math.radians(-60)
        answer=select_body_safe_solution(inverse_kinematics([0,0,-.46],pitch),[0]*4)
        q=answer['selected']['joint_angles_rad']
        transform=forward_kinematics(q)
        self.assertAlmostEqual(sum(q[1:]),pitch)
        self.assertGreater(answer['body_collision_check']['min_clearance_m'],.02)
        height=max(.04,.008+.03*math.cos(pitch)+.0225*abs(math.sin(pitch)))
        # Independent corner enumeration of the finger box relative to grasp_center.
        for x in (-.0225,.0225):
            for z in (-.03,.055):
                corner=transform[:3,:3] @ np.array([x,0,z])
                self.assertGreaterEqual(height+corner[2],.008-1e-9)

    def test_bad_inputs(self):
        for q in ([0,0], [0,float('nan'),0,0]):
            with self.assertRaises(ValueError):arm_body_clearance(q)

if __name__ == '__main__':unittest.main()
