"""Depth decoding and rectified pinhole backprojection, independent of ROS."""
import numpy as np


def decode_depth(message):
    if message.encoding not in ('32FC1', '16UC1'):
        raise ValueError('Unsupported depth encoding: '+message.encoding)
    dtype = np.dtype(('>' if message.is_bigendian else '<') + ('f4' if message.encoding == '32FC1' else 'u2'))
    if message.step < message.width*dtype.itemsize or len(message.data) < message.step*message.height:
        raise ValueError('Invalid depth row stride or buffer length.')
    image = np.ndarray((message.height,message.width),dtype=dtype,buffer=bytes(message.data),
                       strides=(message.step,dtype.itemsize)).astype(float)
    return image if message.encoding == '32FC1' else image*.001


def backproject(depth, u, v, intrinsic):
    if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
        raise ValueError('像素超出图像范围。')
    z = float(depth[v,u])
    if not np.isfinite(z) or z <= 0:
        raise ValueError('该像素没有有效深度，请选择其他位置。')
    k = np.asarray(intrinsic,dtype=float).reshape(3,3)
    if not np.all(np.isfinite(k)) or k[0,0] <= 0 or k[1,1] <= 0:
        raise ValueError('相机内参无效。')
    ray = np.linalg.solve(k,np.array([u,v,1.]))
    return ray * (z/ray[2])
