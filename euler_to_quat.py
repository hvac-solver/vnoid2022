from imageio import imopen
from scipy.spatial.transform import Rotation as R

r = R.from_euler('zyx', angles=[0.3, 0.5, 0.0], degrees=False).as_quat()
print(r)

r = R.from_euler('zyx', angles=[-0.3, 0.5, 0.0], degrees=False).as_quat()
print(r)