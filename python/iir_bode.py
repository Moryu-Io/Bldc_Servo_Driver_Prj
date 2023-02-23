from control import matlab
import numpy as np
import matplotlib.pyplot as plt

Fs = 10000
Fc_hz = 2500
Tc = 1/(Fc_hz)

num = [1]
den = [Tc, 1]

sys = matlab.tf(num, den)
sys_z = matlab.c2d(sys, 1/Fs, method='tustin')

print("********************")
print(sys_z)
print("b0 :",  sys_z.num[0][0][0])
print("b1 :",  sys_z.num[0][0][1])
print("a1 :", -sys_z.den[0][0][1])
print("********************")

matlab.bode(sys)
plt.show()