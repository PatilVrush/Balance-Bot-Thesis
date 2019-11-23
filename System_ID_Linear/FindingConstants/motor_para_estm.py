import numpy as np
import matplotlib.pyplot as plt
import csv
from util import FourierSeries as FS
import pandas as pd


plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"

time1=[]
time2=[]
current1=[]
current2=[]
theta1=[]
theta2=[]
voltage=[]
enc1=[]
enc2=[]
k1=[]
w1=[]
w11=[]

with open('mtd_sin_timed_malloc_sleep_p5.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
		voltage.append(float(row[6]))
		time1.append(float(row[0]))
		enc1.append(int(row[1]))
		current1.append(float(row[4]))
		time2.append(float(row[2]))
		enc2.append(int(row[3]))
		current2.append(float(row[5]))	

time1f = ((np.array(time1)-275.395416).tolist())
time2f = ((np.array(time2)-9280.646507).tolist())

l = (len(enc1))

for i in range(l):
	theta1.append((enc1[i]/979.2)*2*3.14)

for i in range(l):
	theta2.append((enc2[i]/979.2)*2*3.14)

l2 = (len(enc1)-1)

for i in range(l2):
	w1.append((theta1[i+1]-theta1[i])/(time1f[i+1]-time1f[i]))

w1.insert(0,0)

R1 = 7.8

V1  = (((np.array(voltage))*12.1)-(R1*np.array(current1)))

w1_t = (np.array(w1)).T

A_plus1_int = 1/(w1_t.dot(np.array(w1)))

A_plus1 = (np.array(w1_t)*A_plus1_int)
print(A_plus1.shape)
K1 = A_plus1.dot(V1)
print("The value of K1 is:")
print(K1)


torque1 = (np.array(current1))*K1
'''
b1 = A_plus1.dot(torque1)
print("The value of b1 is:")
print(b1)
'''
for i in range(l2):
	w11.append((w1[i+1]-w1[i])/(time1f[i+1]-time1f[i]))

w11.insert(0,0)

'''
const = np.array(w1)*b1

cc=(np.array(torque1)-np.array(const))

w11_t = (np.array(w11)).T
A_plus11_int = 1/(w11_t.dot(np.array(w11)))
A_plus11 = (np.array(w11_t)*A_plus11_int)

Igb1 = A_plus11.dot(cc)
print("The value of Igb1 is:")
print(Igb1)
'''
matrix1 = np.column_stack((w11,w1))
print(matrix1.shape)

A_plus11 = np.linalg.pinv(matrix1)
print(A_plus11.shape)

res1 = A_plus11.dot(torque1)
print(res1)

plt.figure(4)	
plt.plot(time1f,voltage, 'b'  , linewidth=2.0)		
plt.xlabel('Time')
plt.ylabel('Voltage')


plt.figure(5)	
plt.plot(time1f,current1, 'b' , linewidth=2.0)		
plt.xlabel('Time')
plt.ylabel('Current')

plt.figure(6)	
plt.plot(time1f,theta1, 'b' , linewidth=2.0)		
plt.xlabel('Time')
plt.ylabel('Theta')

plt.show()



