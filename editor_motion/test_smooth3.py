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

def make_quaternion_continuous(q):
    q_cont = q.copy()
    for i in range(1, len(q_cont)):
        if np.dot(q_cont[i], q_cont[i-1]) < 0:
            q_cont[i] = -q_cont[i]
    return q_cont

def smooth_quaternion(q, sigma=2.0, size=11):
    q_cont = make_quaternion_continuous(q)
    q_smooth = np.zeros_like(q_cont)
    for i in range(4):
        q_smooth[:, i] = gaussian_filter(q_cont[:, i], sigma=sigma, size=size)
    norms = np.linalg.norm(q_smooth, axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    return q_smooth / norms

# test
T = 100
q = np.random.rand(T, 4)
q /= np.linalg.norm(q, axis=1, keepdims=True)
q_s = smooth_quaternion(q)
if np.all(np.isfinite(q_s)):
    print("SUCCESS")
