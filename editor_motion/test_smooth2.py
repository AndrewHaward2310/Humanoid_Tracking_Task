import numpy as np

def gaussian_kernel(size, sigma=1):
    x = np.linspace(-size // 2, size // 2, size)
    kernel = np.exp(-0.5 * (x / sigma) ** 2)
    return kernel / kernel.sum()

def gaussian_filter(data, sigma=2.0, size=11):
    kernel = gaussian_kernel(size, sigma)
    pad_size = size // 2
    padded = np.pad(data, pad_size, mode='edge')
    return np.convolve(padded, kernel, mode='valid')

def quat_to_euler(q):
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.arcsin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

def euler_to_quat(roll, pitch, yaw):
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return np.stack([w, x, y, z], axis=-1)

np.random.seed(0)
T = 100
base_pos_w = np.random.rand(T, 3)
base_quat_w = np.random.rand(T, 4) 
base_quat_w /= np.linalg.norm(base_quat_w, axis=1, keepdims=True)

# Smooth Z (index 2)
base_pos_w[:, 2] = gaussian_filter(base_pos_w[:, 2], sigma=2.0, size=11)

# Smooth Roll
roll, pitch, yaw = quat_to_euler(base_quat_w)
roll_unwrapped = np.unwrap(roll)
roll_smoothed = gaussian_filter(roll_unwrapped, sigma=2.0, size=11)
smooth_quat = euler_to_quat(roll_smoothed, pitch, yaw)

print("Successfully compiled and ran custom smoothing functions.")
