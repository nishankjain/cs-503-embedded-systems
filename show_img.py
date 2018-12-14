import numpy as np
import matplotlib.pyplot as plt

im = np.loadtxt('./camera_test/test_gray_1.txt')
print(im)
# plt.figure()
plt.imshow(im)
plt.show()