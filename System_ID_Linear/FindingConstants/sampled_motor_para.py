import numpy as np
import matplotlib.pyplot as plt
import csv
from util import FourierSeries as FS
import pandas as pd

plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"
plt.rcParams.update({'font.size': 40})

time1f=[]
time2f=[]
current1=[]
current2=[]
theta1=[]
theta2=[]
voltage=[]
enc1=[]
enc2=[]
time_o=[]
time_t=[]
current_o=[]
current_t=[]
theta_o=[]
theta_t=[]
voltage_n=[]
enc_o=[]
enc_t=[]
w1=[]
w11=[]

with open('mtd_chirp_h.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
		voltage_n.append(float(row[6]))
		time_o.append(float(row[0]))
		enc_o.append(int(row[1]))
		current_o.append(float(row[4]))
		time_t.append(float(row[2]))
		enc_t.append(int(row[3]))
		current_t.append(float(row[5]))	

time_of = ((np.array(time_o)-time_o[0]).tolist())
time_tf = ((np.array(time_t)-time_t[0]).tolist())


l = (len(enc_o))

for i in range(0,l,40):
	enc1.append(enc_o[i])
	enc2.append(enc_t[i])
	current1.append(current_o[i])
	current2.append(current_t[i])
	time1f.append(time_of[i])
	time2f.append(time_tf[i])
	voltage.append(voltage_n[i])


l1 = (len(enc1))

print(l)
print(l1)
for i in range(l1):
	theta1.append((enc1[i]/979.2)*2*3.14)

for i in range(l1):
	theta2.append((enc2[i]/979.2)*2*3.14)

l2 = (len(enc1)-1)

for i in range(l2):
	w1.append((theta1[i+1]-theta1[i])/(time1f[i+1]-time1f[i]))

w1.insert(0,0)

R1 = 7.8

V1  = (((np.array(voltage))*12.4)-(R1*np.array(current1)))

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

print("The value of Igb is:")
print(res1[0])
print("The value of b1 is:")
print(res1[1])

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


'''
data = pd.DataFrame() 
data = data.assign(T1=pd.Series(time1f).values)
data = data.assign(E1=pd.Series(enc1).values)
data = data.assign(T2=pd.Series(time2f).values)
data = data.assign(E2=pd.Series(enc2).values)
data = data.assign(C1=pd.Series(current1).values)
data = data.assign(C2=pd.Series(current2).values)
data = data.assign(V=pd.Series(voltage).values)
data = data.assign(Th1=pd.Series(theta1).values)
data = data.assign(Th2=pd.Series(theta2).values)
data.to_csv("desampled_q_chirp_mtd.csv", index=False)
'''
