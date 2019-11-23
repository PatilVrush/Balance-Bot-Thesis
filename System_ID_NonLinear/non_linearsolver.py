import scipy.signal as SCS
import numpy as np
import scipy.optimize as SO
import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd
from util import *

d2 = loadtxt('augmented_two.csv',delimiter=',')
t2,td2,p2,pd2,tdd2,pdd2,u2,time2 = d2[:,[0,8,2,9,12,13,4,7]].T

tddm=[0.0]*(len(td2))
tddmc=[0.0]*(len(td2))
pddm=[0.0]*(len(td2))
l2 = (len(td2)-1)

tdm = SCS.medfilt(td2,5)
pdm = SCS.medfilt(pd2,5)

for i in range(l2):
        tddm[i] = ((tdm[i+1]-tdm[i-1])/(time2[i+1]-time2[i-1]))

tddm[-1] = (tdm[-1]-tdm[-2])/(time2[-1]-time2[-2])


for i in range(l2):
        pddm[i] = ((pdm[i+1]-pdm[i-1])/(time2[i+1]-time2[i-1]))

pddm[-1] = (pdm[-1]-pdm[-2])/(time2[-1]-time2[-2])

tddmm = SCS.medfilt(tddm,5)
pddmm = SCS.medfilt(pddm,5)
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


