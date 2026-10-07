import unittest
import numpy as np
from ghost_sim.navigation.planner3d import Planner

class PlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.p=Planner()
    def test_clear_vertical_path(self):
        route=self.p.plan([0,0,1.2],[0,0,2.2])
        self.assertEqual(len(route),2)
    def test_over_box_avoids_swept_collision(self):
        a,b=[.8,1,.5],[2.8,1,.5]
        self.assertFalse(self.p.segment_clear(a,b))
        route=self.p.plan(a,b)
        self.assertGreater(len(route),2)
        self.assertTrue(all(self.p.segment_clear(x,y) for x,y in zip(route,route[1:])))
    def test_invalid_goals(self):
        for goal in [[1.8,1,.3],[0,0,3.2],[np.nan,0,1],[0,0,.1]]:
            with self.assertRaises(ValueError):self.p.plan([0,0,1.2],goal)
    def test_middle_of_long_segment_is_checked(self):
        self.assertFalse(self.p.segment_clear([-3,1.2,.5],[3,1.2,.5]))
        self.assertFalse(self.p.segment_clear([3,1.2,.5],[-3,1.2,.5]))
        self.assertTrue(self.p.segment_clear([-3,1.2,2],[3,1.2,2]))
    def test_restart_from_safe_conservative_blocked_cell(self):
        start=[2.3487700886,.4694400431,.5744717312]
        self.assertFalse(self.p.free(start))
        self.assertTrue(self.p.segment_clear(start,start))
        route=self.p.plan(start,[0,0,1.2])
        self.assertTrue(all(self.p.segment_clear(a,b) for a,b in zip(route,route[1:])))
    def test_stationary_goal(self):
        route=self.p.plan([0,0,1.2],[0,0,1.2])
        self.assertTrue(self.p.segment_clear(*route))

if __name__=='__main__':unittest.main()
