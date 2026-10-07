import unittest
import numpy as np
from ghost_sim.control.ik_execute import interpolation, execute_target


class IKExecutionTests(unittest.TestCase):
    def test_interpolation_and_speed_bound(self):
        start = np.array([0,-.8,-.6,1.4])
        target = np.array([.3,-1.,-.2,1.2])
        duration = max(4., 1.5*np.max(np.abs(target-start))/.35)
        samples = np.array([interpolation(start,target,t,duration) for t in np.linspace(0,duration,1001)])
        np.testing.assert_allclose(samples[0],start)
        np.testing.assert_allclose(samples[-1],target)
        self.assertLessEqual(np.max(np.abs(np.diff(samples,axis=0)/(duration/1000))),.35+1e-8)
        self.assertTrue(np.all(samples>=np.minimum(start,target)-1e-12))
        self.assertTrue(np.all(samples<=np.maximum(start,target)+1e-12))
        np.testing.assert_allclose(interpolation(start,target,duration+2,duration),target)

    def test_bad_duration_before_ros(self):
        for duration in (0,-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):
                execute_target([0,0,0],0,duration)
