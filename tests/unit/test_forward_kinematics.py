"""Compare DH with the independent URDF joint tree, not another DH table."""
import unittest
import xml.etree.ElementTree as E
import numpy as np
from ghost_sim.paths import MODEL
from ghost_sim.kinematics.forward import forward_kinematics, load_parameters


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*skew + (1-np.cos(angle))*(skew @ skew)


def urdf_fk(q):
    root = E.parse(MODEL).getroot()
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    values = dict(zip(['tail_yaw','tail_shoulder','tail_elbow','tail_wrist'], q))
    chain = []
    child = 'grasp_center'
    while child != 'base_link':
        joint = joints[child]
        chain.append(joint)
        child = joint.find('parent').get('link')
    result = np.eye(4)
    for joint in reversed(chain):
        origin = joint.find('origin')
        local = np.eye(4)
        if origin is not None:
            local[:3, 3] = np.fromstring(origin.get('xyz','0 0 0'), sep=' ')
            r, p, y = np.fromstring(origin.get('rpy','0 0 0'), sep=' ')
            local[:3, :3] = rotation([0,0,1],y) @ rotation([0,1,0],p) @ rotation([1,0,0],r)
        result = result @ local
        if joint.get('type') == 'revolute':
            motion = np.eye(4)
            motion[:3, :3] = rotation(np.fromstring(joint.find('axis').get('xyz'),sep=' '),values[joint.get('name')])
            result = result @ motion
    return result


class ForwardKinematicsTests(unittest.TestCase):
    def test_zero_pose_geometry(self):
        expected = np.eye(4)
        expected[:3, 3] = [-.33, 0, -.66]
        np.testing.assert_allclose(forward_kinematics([0,0,0,0]),expected,atol=1e-12)

    def test_ready_pose_points_down(self):
        transform = forward_kinematics([0,-.8,-.6,1.4])
        np.testing.assert_allclose(transform[:3,:3],np.eye(3),atol=1e-12)
        np.testing.assert_allclose(transform,urdf_fk([0,-.8,-.6,1.4]),atol=1e-12)

    def test_200_urdf_configurations(self):
        random = np.random.default_rng(42)
        for q in random.uniform([-1.3,-1.6,-1.8,-2.2],[1.3,.7,1.8,2.2],size=(200,4)):
            np.testing.assert_allclose(forward_kinematics(q),urdf_fk(q),atol=1e-12)

    def test_bad_angles(self):
        for q in ([0,0,0], [0,0,0,0,0], [0,np.nan,0,0], [0,np.inf,0,0]):
            with self.assertRaises(ValueError):
                forward_kinematics(q)


if __name__ == '__main__':
    unittest.main()
