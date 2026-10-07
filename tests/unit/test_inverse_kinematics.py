"""IK reachability, branches, limits, singular cases and independent URDF checks."""
import math
import unittest
import numpy as np
from ghost_sim.kinematics.inverse import inverse_kinematics, LIMITS
from test_forward_kinematics import urdf_fk


class InverseKinematicsTests(unittest.TestCase):
    def test_ready_two_branches_and_selection(self):
        q = [0,-.8,-.6,1.4]
        p = urdf_fk(q)[:3,3]
        answer = inverse_kinematics(p,0,q)
        self.assertEqual(answer['solution_count'],2)
        np.testing.assert_allclose(answer['selected']['joint_angles_rad'],q,atol=1e-8)
        other = answer['solutions'][1]['joint_angles_rad']
        chosen = inverse_kinematics(p,0,other)
        np.testing.assert_allclose(chosen['selected']['joint_angles_rad'],other,atol=1e-8)

    def test_500_urdf_targets(self):
        rng = np.random.default_rng(137)
        for q in rng.uniform(LIMITS[:,0],LIMITS[:,1],size=(500,4)):
            target = urdf_fk(q)
            answer = inverse_kinematics(target[:3,3],sum(q[1:]),q)
            selected = answer['selected']['joint_angles_rad']
            np.testing.assert_allclose(selected,q,atol=1e-7)
            for solution in answer['solutions']:
                solved = np.array(solution['joint_angles_rad'])
                self.assertTrue(np.all(solved >= LIMITS[:,0]-1e-9))
                self.assertTrue(np.all(solved <= LIMITS[:,1]+1e-9))
                np.testing.assert_allclose(urdf_fk(solved)[:3,3],target[:3,3],atol=1e-8)
                self.assertLess(abs(math.sin(sum(solved[1:])-sum(q[1:]))),1e-8)

    def test_signed_radius_behind(self):
        q = [.2,.5,.3,-.8]
        p = urdf_fk(q)[:3,3]
        self.assertLess(p[0],-.33)
        np.testing.assert_allclose(inverse_kinematics(p,0,q)['selected']['joint_angles_rad'],q,atol=1e-8)

    def test_yaw_axis_and_straight_arm(self):
        q = [.7,0,0,0]
        result = inverse_kinematics([-.33,0,-.66],0,q)
        self.assertTrue(result['yaw_underdetermined'])
        self.assertTrue(result['selected']['near_singular'])
        self.assertEqual(result['solution_count'],1)
        np.testing.assert_allclose(result['selected']['joint_angles_rad'],q,atol=1e-7)

    def test_joint_limit_boundaries(self):
        for q in (LIMITS[:,0],LIMITS[:,1]):
            answer = inverse_kinematics(urdf_fk(q)[:3,3],sum(q[1:]),q)
            np.testing.assert_allclose(answer['selected']['joint_angles_rad'],q,atol=1e-7)

    def test_unreachable_and_limit_rejection(self):
        with self.assertRaisesRegex(ValueError,'几何可达'):
            inverse_kinematics([3,0,0])
        q = [math.pi/2,-.8,-.6,1.4]
        with self.assertRaisesRegex(ValueError,'关节限位'):
            inverse_kinematics(urdf_fk(q)[:3,3],0)

    def test_invalid_inputs(self):
        for p in ([0,0], [0,float('nan'),0], [0,0,float('inf')]):
            with self.assertRaises(ValueError):inverse_kinematics(p)
        with self.assertRaises(ValueError):inverse_kinematics([0,0,0],float('nan'))
        with self.assertRaises(ValueError):inverse_kinematics([0,0,0],0,[0,0,0])
        with self.assertRaises(ValueError):inverse_kinematics([0,0,0],0,[4,0,0,0])


if __name__ == '__main__':
    unittest.main()
