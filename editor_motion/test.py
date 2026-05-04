import numpy as np
a = np.load('original.npz'); b = np.load('saved.npz')
print('keys:', a.files, b.files)
# compare shapes/dtypes
for k in a.files:
    print(k, a[k].shape, a[k].dtype, '==', (k in b.files and b[k].shape==a[k].shape))
# check base quat ordering and norm
if 'base_quat_w' in a.files:
    print('orig quat[0]:', a['base_quat_w'][0])
if 'base_quat_w' in b.files:
    print('saved quat[0]:', b['base_quat_w'][0])
# check joint name mapping and first-frame values
print('joint names equal:', np.array_equal(a['joint_names'], b['joint_names']))
print('first frame joint_pos equal:', np.allclose(a['joint_pos'][0], b['joint_pos'][0]))