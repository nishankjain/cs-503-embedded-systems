import numpy as np
import matplotlib.pyplot as plt

im = np.loadtxt('test_gray_1')
print(im)
# plt.figure()
plt.imshow(im)
plt.show()