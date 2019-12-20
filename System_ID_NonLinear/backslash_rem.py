import scipy.signal as SCS
import numpy as np
import scipy.optimize as SO
import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd
from util import *
from pandas import DataFrame	

d2 = loadtxt('augmented_two.csv',delimiter = ',')
t,td2,p,pd2,tdd2,pdd2,u2,time2 = d2[:,[0,8,2,9,12,13,4,7]].T #0,8,2,9

tdd=[0.0]*(len(td2))
pdd=[0.0]*(len(td2))
l2 = (len(td2)-1)

td = SCS.medfilt(td2,5)
pd = SCS.medfilt(pd2,5)

for i in range(l2):
        tdd[i] = ((td[i+1]-td[i-1])/(time2[i+1]-time2[i-1]))

tdd[-1] = (td[-1]-td[-2])/(time2[-1]-time2[-2])

for i in range(l2):
        pdd[i] = ((pd[i+1]-pd[i-1])/(time2[i+1]-time2[i-1]))

pdd[-1] = (pd[-1]-pd[-2])/(time2[-1]-time2[-2])

tddo = SCS.medfilt(tdd,5)
pddo = SCS.medfilt(pdd,5)
u2o = SCS.medfilt(u2,5)

pdm = [0.0]*(len(td2))
pddmm = [0.0]*(len(td2))
tdm = [0.0]*(len(td2))
tddmm = [0.0]*(len(td2))
p2 = [0.0]*(len(td2))
t2 = [0.0]*(len(td2))
u2m = [0.0]*(len(td2))


hye=0
bye=0
sign = lambda x: (1, -1)[x < 0]

for i in range(len(p)):
	if(bye< (len(p)-1)):
		if ((sign(pd[bye])) == (sign(pd[bye+1]))):
			pdm[hye]= pd[bye]
			tdm[hye]= td[bye]
			p2[hye]= p[bye]
			t2[hye]= t[bye]
			u2m[hye]= u2o[bye]
			pddmm[hye]= pddo[bye]
			tddmm[hye]= tddo[bye]
			bye=bye+1
			hye=hye+1
	
		else:
			hye=hye-7
			bye=bye+8
	else:
		break


'''
for i in range(len(p2)):
		pdm[i]= pd[i]
		tdm[i]= td[i]
		p2[i]= p[i]
		t2[i]= t[i]
		u2m[i]= u2o[i]
		pddmm[i]= pddo[i]
		tddmm[i]= tddo[i]
'''			

pdm = np.array(pdm)
pddmm = np.array(pddmm)
tdm = np.array(tdm)
tddmm = np.array(tddmm)
p2 = np.array(p2)
t2 = np.array(t2)
u2m = np.array(u2m)

'''
dict = {'tdm' : tdm, 'pdm': pdm, 'tddmm' : tddmm, 'pddmm' : pddmm}
dp = DataFrame(dict, columns = ['tdm' , 'pdm' ,'tddmm' , 'pddmm'])
export_csv1 = dp.to_csv ('med_all.csv', index = None, header='False') 
'''

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
	   #res = c_[(a1*pddmm+a2*tddmm*cos(t2))+(a2*pddmm*cos(t2)+a3*tddmm),(a4*sin(t2))+(a2*tdm*tdm*sin(t2))]
           return res   #res.flatten()

#print(SO.fmin(lambda x : sum(myFunction_nl(x)**2), [0.002,0.0046,0.018,1.13,0.02],full_output=1))

#f = myFunction_nl([0.0025,0.00099,0.0077,0.164,0.0212])
f = myFunction_nl([0.0021,0.0046,0.018,1.134,0.0212])
g = myFunction_lin([0.0025,0.00099,0.0077,0.164,0.0212])
'''
plt.figure(1)
plt.plot(g[:,0], lw=2,c = 'r')
plt.plot(f[:,0], lw=2,c = 'b')
plt.plot(f[:,2], lw=2,c = 'c')
'''

plt.figure(2)
plt.plot(f[:,0],f[:,2] ,'.r',label='Backlash Rem')
plt.show()

plt.figure(3)
plt.plot(f[:,1],f[:,3] ,'.')
plt.show()

from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
a = fig.add_subplot(111,projection='3d')
a.scatter( f[:,1], f[:,2], f[:,0])
plt.show()
