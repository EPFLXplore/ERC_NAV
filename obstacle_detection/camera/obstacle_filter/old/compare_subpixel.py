import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

folder = 'noSubpixel'

# gray = np.load('images/' + folder + '/rgb/gray_data_' + str(200) + '.npy')

# rgb = np.load('images/' + folder + '/rgb/rgb_data_' + str(200) + '.npy')
# rgb = np.transpose(rgb, (1, 2, 0))

depth_no_subpixel = np.load('images/' + folder + '/depth/depth_data_' + str(200) + '.npy')

folder2 = 'Subpixel'
depth_subpixel = np.load('images/' + folder2 + '/depth/depth_data_' + str(120) + '.npy')

plt.figure(figsize=(20, 10))
plt.imshow(depth_no_subpixel)

plt.figure(figsize=(20, 10))
plt.imshow(depth_subpixel)
# plt.imshow(rgb)
plt.show()


