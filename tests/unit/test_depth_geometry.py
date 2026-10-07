import unittest
from types import SimpleNamespace
import numpy as np
from ghost_sim.perception.depth_geometry import decode_depth, backproject


class DepthGeometryTests(unittest.TestCase):
    def test_pinhole(self):
        depth=np.ones((3,3))*2
        k=[[2,0,1],[0,2,1],[0,0,1]]
        np.testing.assert_allclose(backproject(depth,1,1,k),[0,0,2])
        np.testing.assert_allclose(backproject(depth,2,0,k),[1,-1,2])

    def test_padded_big_endian_mm(self):
        row=np.array([1000,2000,9999],dtype='>u2').tobytes()
        m=SimpleNamespace(encoding='16UC1',is_bigendian=True,width=2,height=2,step=6,data=row*2)
        np.testing.assert_allclose(decode_depth(m),[[1,2],[1,2]])

    def test_float_depth_and_invalid(self):
        a=np.array([[1,np.nan],[np.inf,0]],dtype='<f4')
        m=SimpleNamespace(encoding='32FC1',is_bigendian=False,width=2,height=2,step=8,data=a.tobytes())
        d=decode_depth(m)
        self.assertEqual(d[0,0],1)
        for u,v in [(1,0),(0,1),(1,1),(-1,0),(2,0)]:
            with self.assertRaises(ValueError):backproject(d,u,v,np.eye(3))
        m.step=2
        with self.assertRaises(ValueError):decode_depth(m)
