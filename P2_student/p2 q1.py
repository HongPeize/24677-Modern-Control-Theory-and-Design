import numpy as np
import control
from scipy import signal, linalg
import matplotlib.pyplot as plt

lr = 1.39
lf = 1.55
Ca = 20000
Iz = 25854
m = 1888.6
g = 9.81
xdot=8 #2,5,8m/s depends on questions

A = np.array([[0, 1, 0, 0], [0, -4*Ca / (m * xdot), 4*Ca / m, -(2*Ca*(lr - lf))/(m*xdot)], [0, 0, 0, 1], [0, -(2*Ca*(lf - lr)) / (Iz * xdot), (2*Ca*(lf - lr)) / Iz, (-2*Ca*(np.power(lf, 2) + np.power(lr, 2))) / (Iz * xdot)]])
B = np.array([[0, 0], [2*Ca / m, 0], [0, 0], [(2 * Ca* lf) / Iz, 0]])
C=np.identity(4)
controlability_matrix=control.ctrb(A, B)
observability_matrix=control.obsv(A, C)
print(controlability_matrix)
print(observability_matrix)
print("controllable?", np.linalg.matrix_rank(controlability_matrix))
print("observable?", np.linalg.matrix_rank(observability_matrix))

log_list=[]
i_list=[]
list_pole1=[]
list_pole2=[]
list_pole3=[]
list_pole4=[]

for i in range(1,41):
    xdot=i
    A = np.array([[0, 1, 0, 0], [0, -4*Ca / (m * xdot), 4*Ca / m, -(2*Ca*(lf - lr))/(m*xdot)], [0, 0, 0, 1], [0, -(2*Ca*(lf - lr)) / (Iz * xdot), (2*Ca*(lf - lr)) / Iz, (-2*Ca*(np.power(lf, 2) + np.power(lr, 2))) / (Iz * xdot)]])
    B = np.array([[0], [2*Ca / m], [0], [(2 * Ca* lf) / Iz]])
    C = np.array([1,1,1,1])
    D=np.array([0])
    controlability_matrix = control.ctrb(A, B)
    _, s, _ = np.linalg.svd(controlability_matrix)
    log_list.append(np.log10(np.max(s)/np.min(s)))
    i_list.append(i)

    sys = control.StateSpace(A, B, C, D)
    p = control.pole(sys)
    #print(p)
    list_pole1.append(np.real(p[0]))
    list_pole2.append(np.real(p[1]))
    list_pole3.append(np.real(p[2]))
    list_pole4.append(np.real(p[3]))

plt.plot(i_list,log_list)
plt.ylabel('log')
plt.xlabel('V(m/s)')
plt.show()

plt.plot(i_list,list_pole1)
plt.ylabel('Re(pole 1)')
plt.xlabel('V(m/s)')
plt.show()

plt.plot(i_list,list_pole2)
plt.ylabel('Re(pole 2)')
plt.xlabel('V(m/s)')
plt.show()

plt.plot(i_list,list_pole3)
plt.ylabel('Re(pole 3)')
plt.xlabel('V(m/s)')
plt.show()

plt.plot(i_list,list_pole4)
plt.ylabel('Re(pole 4)')
plt.xlabel('V(m/s)')
plt.show()


