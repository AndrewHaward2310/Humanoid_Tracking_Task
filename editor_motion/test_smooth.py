import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation as R

np.random.seed(0)
T = 100
base_pos_w = np.random.rand(T, 3)
base_quat_w = np.random.rand(T, 4) 
# normalize quat
base_quat_w /= np.linalg.norm(base_quat_w, axis=1, keepdims=True)

# Smooth Z (index 2)
base_pos_w[:, 2] = gaussian_filter1d(base_pos_w[:, 2], sigma=2.0)

# Smooth Roll
# scipy Rotation uses scalar-last (x, y, z, w)
# base_quat_w uses scalar-first (w, x, y, z)
quats_xyzw = base_quat_w[:, [1, 2, 3, 0]]
r = R.from_quat(quats_xyzw)
euler = r.as_euler('xyz', degrees=False) # x is roll, y is pitch, z is yaw
# Unwrap to avoid jumps
euler[:, 0] = np.unwrap(euler[:, 0])
euler[:, 0] = gaussian_filter1d(euler[:, 0], sigma=2.0)
r_smooth = R.from_euler('xyz', euler, degrees=False)
smooth_quats_xyzw = r_smooth.as_quat()
base_quat_w[:, 0] = smooth_quats_xyzw[:, 3]
base_quat_w[:, 1:] = smooth_quats_xyzw[:, :3]

print("Successfully compiled and ran.")
