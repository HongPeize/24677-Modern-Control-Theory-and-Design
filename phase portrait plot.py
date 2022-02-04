import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#original system
def o(X):
    x1, x2 = X
    sys=[x2-x1*x2**2, -x1**3]
    return sys

#linearized system
def l(X):
    x1, x2 = X
    sys = [x2,0]
    return sys

def sys_plot(sys):
  x1 = np.arange(-8, 8, 0.5)
  x2 = np.arange(-8, 8, 0.5)

  X1, X2 = np.meshgrid(x1, x2)
  t = 0
  u, v = np.zeros(X1.shape), np.zeros(X2.shape)
  NI, NJ = X1.shape
  for i in range(NI):
      for j in range(NJ):
        x = X1[i, j]
        y = X2[i, j]
        xprime = sys([x, y])
        u[i, j] = xprime[0]
        v[i, j] = xprime[1]
  return x1,x2,u,v

#original system
x1_o, x2_o, u_o, v_o=sys_plot(o)
Q = plt.quiver(x1_o, x2_o, u_o, v_o, color='r')
plt.xlabel('$x_1$')
plt.ylabel('$x_2$')
plt.xlim([-8, 8])
plt.ylim([-8, 8])
plt.title('original system')
plt.show()

#linearized system
x1_l, x2_l, u_l, v_l=sys_plot(l)
Q = plt.quiver(x1_l, x2_l, u_l, v_l, color='b')
plt.xlabel('$x_1$')
plt.ylabel('$x_2$')
plt.xlim([-8, 8])
plt.ylim([-8, 8])
plt.title('linearized system')
plt.show()
