#kalman_filtering
import scipy.signal as SCS
import numpy as np
import scipy.optimize as SO
import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd
from util import *
import numpy as np
import pandas as pd
from numpy.linalg import inv


ta=[]
tda=[]
tdda=[]

pa=[]
pda=[]
pdda=[]

d2 = loadtxt('augmented_two.csv',delimiter=',')
t2,td2,p2,pd2,tdd2,pdd2,u2,time2 = d2[:,[0,8,2,9,12,13,4,6]].T

prv_timet = 613.225
prv_timep = 613.225

xt = np.array([
        [-0.053],
        [0],
        [0]
        ])

xp = np.array([
        [-0.006],
        [0],
        [0]
        ])

'''
d2 = loadtxt('testing_gyro22.csv',delimiter=',')

t2,td2,p2,pd2,tdd2,pdd2,u2,time2 = d2[:,[0,1,2,9,12,13,4,20]].T

x = np.array([
        [0.291],
        [0],
        [0]
        ])

prv_time = 1598.711
'''

l2=(len(t2)-1)
tdc=[0.0]*len(t2)
pdc=[0.0]*len(t2)

for i in range(l2):
        tdc[i] = ((t2[i+1]-t2[i])/(time2[i+1]-time2[i]))

tdc[-1] = (t2[-1]-t2[-2])/(time2[-1]-time2[-2])

for i in range(l2):
        pdc[i] = ((p2[i+1]-p2[i])/(time2[i+1]-time2[i]))

pdc[-1] = (p2[-1]-p2[-2])/(time2[-1]-time2[-2])


'''
Pt = np.array([
        [1, 0, 0],
        [0, 100, 0],
        [0, 0, 100]])


At = np.array([
        [1.0, 1.0, 1],
        [0, 1.0, 1.0],
        [0, 0, 1.0]])

Ht = np.array([
        [1.0, 0, 0]])

It = np.identity(3)

zt = np.zeros([1, 1])

Rt = 0.1

Qt = np.zeros([3, 3])
Qt[0][0] = 0.001
Qt[1][1] = 0.01
Qt[2][2] = 200



def predict_t():
    # Predict Step
    global xt, Pt, Qt
    xt = np.matmul(At, xt)
    Att = np.transpose(At)
    Pt = np.add(np.matmul(At, np.matmul(Pt, Att)), Qt)

def update_t(z):
    global xt, Pt    
    # Measurement update step
    Yt = np.subtract(z, np.matmul(Ht, xt))
    Htt = np.transpose(Ht)
    St = np.add(np.matmul(Ht, np.matmul(Pt, Htt)), Rt)
    Kt = np.matmul(Pt, Htt)
    Sit = inv(St)
    Kt = np.matmul(Kt, Sit)
    
    # New state
    xt = np.add(xt, np.matmul(Kt, Yt))
    Pt = np.matmul(np.subtract(It ,np.matmul(Kt, Ht)), Pt)
    ta.append(xt[0][0])
    tda.append(xt[1][0])
    tdda.append(xt[2][0])

for i in range (len(t2)):
	
	cur_timet = time2[i]
        dtt = cur_timet - prv_timet
        prv_timet = cur_timet

	dt_2t = dtt * dtt

	At[0][1] = dtt
        At[0][2] = 0.5*dt_2t
	At[1][2] = dtt

        #Updating sensor readings
        zt = t2[i]

	predict_t()
        update_t(zt)


'''

Pp = np.array([
        [1, 0, 0],
        [0, 100, 0],
        [0, 0, 100]])


Ap = np.array([
        [1.0, 1.0, 1],
        [0, 1.0, 1.0],
        [0, 0, 1.0]])

Hp = np.array([
        [1.0, 0, 0]])

Ip = np.identity(3)

zp = np.zeros([1, 1])

Rp = 0.01

Qp = np.zeros([3, 3])
Qp[0][0] = 1
Qp[1][1] = 1
Qp[2][2] = 100


def predict_p():
    # Predict Step
    global xp, Pp, Qp
    xp = np.matmul(Ap, xp)
    Atp = np.transpose(Ap)
    Pp = np.add(np.matmul(Ap, np.matmul(Pp, Atp)), Qp)

def update_p(z):
    global xp, Pp    
    # Measurement update step
    Yp = np.subtract(z, np.matmul(Hp, xp))
    Htp = np.transpose(Hp)
    Sp = np.add(np.matmul(Hp, np.matmul(Pp, Htp)), Rp)
    Kp = np.matmul(Pp, Htp)
    Sip = inv(Sp)
    Kp = np.matmul(Kp, Sip)
    
    # New state
    xp = np.add(xp, np.matmul(Kp, Yp))
    Pp = np.matmul(np.subtract(Ip ,np.matmul(Kp, Hp)), Pp)
    pa.append(xp[0][0])
    pda.append(xp[1][0])
    pdda.append(xp[2][0])

for i in range (len(t2)):
	
	cur_timep = time2[i]
        dtp = cur_timep - prv_timep
        prv_timep = cur_timep

	dt_2p = dtp * dtp

	Ap[0][1] = dtp
        Ap[0][2] = 0.5*dt_2p
	Ap[1][2] = dtp

        #Updating sensor readings
        zp = p2[i]

	predict_p()
        update_p(zp)


'''

plt.figure(1)
plt.plot(ta, lw=2,c = 'b', label = 'Estimated')
plt.plot(t2, lw=2,c = 'c', label = 'GT')
plt.legend()

plt.figure(2)
plt.plot(tda, lw=2,c = 'g', label = 'Estimated')
plt.plot(td2, lw=2,c = 'y', label = 'GT')
#plt.plot(tdc, lw=2,c = 'c', label = 'C')
plt.legend()

plt.figure(3)
plt.plot(tdda, lw=2,c = 'r', label = 'Estimated')
plt.plot(tdd2, lw=2,c = 'm', label = 'GT')
plt.legend()

plt.figure(4)
plt.plot(ta, lw=2,c = 'g', label = 'T')
plt.plot(tda, lw=2,c = 'y', label = 'TD')
plt.plot(tdda, lw=2,c = 'c', label = 'TDD')
plt.legend()
plt.show()
'''

plt.figure(1)
plt.plot(pa, lw=2,c = 'b', label = 'Estimated')
plt.plot(p2, lw=2,c = 'c', label = 'GT')
plt.legend()

plt.figure(2)
plt.plot(pda, lw=2,c = 'g', label = 'Estimated')
plt.plot(pd2, lw=2,c = 'y', label = 'GT')
#plt.plot(pdc, lw=2,c = 'c', label = 'C')
plt.legend()

plt.figure(3)
plt.plot(pdda, lw=2,c = 'r', label = 'Estimated')
plt.plot(pdd2, lw=2,c = 'm', label = 'GT')
plt.legend()

plt.figure(4)
plt.plot(pa, lw=2,c = 'g', label = 'P')
plt.plot(pda, lw=2,c = 'y', label = 'PD')
plt.plot(pdda, lw=2,c = 'c', label = 'PDD')
plt.legend()
plt.show()


#**************************** RHS LHS COMPARISON **********************#

'''
t2=ta
tdm = tda
tddmm=tdda

p2 = pa
pdm = pda
pddmm = pdda

u2m = SCS.medfilt(u2,5)

def myFunction_lin(z):
           a1,a2,a3,a4,b2 = abs(np.asarray(z))
           b1=1 
           v = b2*tdm-b2*pdm+b1*u2m
           res = c_[a1*pddmm+a2*tddmm,a2*pddmm+a3*tddmm-a4*t2,v,-v]
           return res


def myFunction_nl(z):
           a1,a2,a3,a4,b2 = abs(np.asarray(z))
           b1=1 
           v = b2*tdm-b2*pdm+b1*u2m
           res = c_[a1*pddmm+a2*tddmm*cos(t2)-a2*tdm*tdm*sin(t2),a2*pddmm*cos(t2)+a3*tddmm-a4*sin(t2),v,-v]
           return res   #res.flatten()

#print(SO.fmin(lambda x : sum(myFunction_nl(x)**2), [0.002,0.0046,0.018,1.13,0.02],full_output=1))


f = myFunction_nl([0.0025,0.00099,0.0077,0.164,0.0212])
g = myFunction_lin([0.0025,0.00099,0.0077,0.164,0.0212])

plt.figure(1)
plt.plot(g[:,0], lw=2,c = 'r')
plt.plot(f[:,0], lw=2,c = 'b')
plt.plot(f[:,2], lw=2,c = 'c')
plt.show()

a1,a2,a3,a4,b1,b2 = 0.0025,0.00099,0.0077,0.164,1,0.0212

tdde=[0.0]*(len(td2))
pdde=[0.0]*(len(td2))

vt = b2*tdm-b2*pdm+b1*u2m
l3 = (len(t2))

RT = ((a2*a2*cos(t2)*cos(t2))-(a3*a1))

for i in range(l3):
	pdde[i] =  (    (-1*(a2*cos(t2[i])+a3)*vt[i])  +    (  (a2*sin(t2[i])) * (cos(t2[i])*a4-a3*tdm[i]*tdm[i]) ) )/RT[i]

for i in range(l3):
	tdde[i] = (     ( (a2*cos(t2[i])+a1)*vt[i])   -   ( (sin(t2[i]) ) * (a1*a4-a2*a2*tdm[i]*tdm[i]*cos(t2[i]))  ))/RT[i]

tdde = np.array(tdde) #  *0.4)+15
pdde = np.array(pdde)  # *0.6-15

print(tdde.shape)

plt.figure(2)
plt.plot(tdde, lw=2,c = 'r', label = 'tdd-estimated')
plt.plot(tddmm, lw=2,c = 'b', label = 'tdd-measured')	
plt.legend()

plt.figure(3)
plt.plot(tdde-tddmm, lw=2,c = 'r', label = 'errort')
plt.legend()

plt.figure(4)
plt.plot(pdde, lw=2,c = 'r', label = 'pdd-estimated')
plt.plot(pddmm, lw=2,c = 'b', label = 'pdd-measured')
plt.legend()

plt.figure(5)
plt.plot(pdde-pddmm, lw=2,c = 'r', label = 'errorp')
plt.legend()

st = c_[t2,tdm,p2,pdm]
errt = tdde-tddmm
errp = pdde-pddmm

figure(6)
Rt = asarray([ dot(errt[k:],st[:-k,:])*errt.size/float(errt.size-k) for k in range(1,500)])
plt.plot(Rt,'.-')
plt.legend(['t2','tdm','p2','pdm'])

figure(7)
Rp = asarray([ dot(errp[k:],st[:-k,:])*errp.size/float(errp.size-k) for k in range(1,500)])
plt.plot(Rp,'.-')
plt.legend(['t2','tdm','p2','pdm'])

figure(8)
plt.plot(tddmm,tdde,'.')

figure(9)
plt.plot(pddmm,pdde,'.')

figure(10)
plt.plot(f[:,0],'.-b')
plt.plot(g[:,0],'.-r')

figure(11)
plt.plot(f[:,1],'.-b',label='NL')
plt.plot(g[:,1],'.-r',label='L')
plt.show()
'''






