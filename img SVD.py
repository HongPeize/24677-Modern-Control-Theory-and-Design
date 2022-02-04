import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import time
from PIL import Image

img = mpimg.imread('/Users/jameshong/Documents/CMU year1 sem1 /24677 Modern Control Theory/CMU_Grayscale.png')
print(img)
u, s, vh = np.linalg.svd(img, full_matrices=True)
print(u.shape, s.shape, vh.shape)

#50%
i=216
reconstimg50 = np.matrix(u[:, :i]) * np.diag(s[:i]) * np.matrix(vh[:i, :])
plt.imshow(reconstimg50, cmap='gray')
title1 = "n = %s" % i
plt.title(title1)
plt.show()

#10%
j=43
reconstimg10 = np.matrix(u[:, :j]) * np.diag(s[:j]) * np.matrix(vh[:j, :])
plt.imshow(reconstimg10, cmap='gray')
title2 = "n = %s" % j
plt.title(title2)
plt.show()

#5%
k=22
reconstimg5 = np.matrix(u[:, :k]) * np.diag(s[:k]) * np.matrix(vh[:k, :])
plt.imshow(reconstimg5, cmap='gray')
title3 = "n = %s" % k
plt.title(title3)
plt.show()
