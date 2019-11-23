import pandas 
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as SI
import csv
from util import *
from pandas import DataFrame

plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"
plt.rcParams.update({'font.size': 40})

theta=[]
theta_dot=[]
phi=[]
phi_dot=[]
time=[]
tnd = []
 
with open('augmented_two.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
		theta.append(float(row[0]))
		theta_dot.append(float(row[1]))
		phi.append(float(row[2]))
		phi_dot.append(float(row[3]))
		time.append(float(row[7]))
	
tddm=[0.0]*(len(td2))
pddm=[0.0]*(len(td2))


l2 = (len(td2)-1)

A,B = SI.butter(2,0.2)
td = SI.filtfilt(A,B,theta_dot)
pd = SI.filtfilt(A,B,phi_dot)

tdm = SI.medfilt(td,5)
pdm = SI.medfilt(pd,5)

#td = theta_dot
#pd = phi_dot


for i in range(l2):
	tddm[i] = ((tdm[i+1]-tdm[i])/(time[i+1]-time[i]))

tddm[-1] = (tdm[-1]-tdm[-2])/(time[-1]-time[-2])

#tdds =  SI.filtfilt(A,B,tdd)
tddmm =  SI.medfilt(tddm,5)

for i in range(l2):
	pddm[i] = ((pdm[i+1]-pdm[i])/(time[i+1]-time[i]))

pddm[-1] = (pdm[-1]-pdm[-2])/(time[-1]-time[-2])

#tdds =  SI.filtfilt(A,B,tdd)
pddmm =  SI.medfilt(pddm,5)
pddmms =  SI.filtfilt(A,B,pddmm)

for i in range(l2):
	pdd[i] = ((pd[i+1]-pd[i])/(time[i+1]-time[i]))

pdd[-1] = (pd[-1]-pd[-2])/(time[-1]-time[-2])
pdds =  SI.filtfilt(A,B,pdd)


dict = {'tdm' : tdm, 'pdm': pdm, 'tddmm' : tddmm, 'pddmm' : pddmm}
dp = DataFrame(dict, columns = ['tdm' , 'pdm' ,'tddmm' , 'pddmm'])
export_csv1 = dp.to_csv ('med_all.csv', index = None, header='False') 

'''
pp = DataFrame(pd)
export_csv2 = pp.to_csv ('pd.csv', index = None, header=True) 
'''

plt.figure(4)	
plt.plot(td, 'b.-'  , linewidth=2.0 , label='Theta_DOT_BW')
plt.plot(tdm, 'g.-'  , linewidth=2.0 ,label='Theta_Dot_MED')
plt.xlabel('Time')
plt.legend()

plt.figure(6)	
plt.plot(tddm, 'g.-'  , linewidth=2.0, label='Theta_DDM')
plt.plot(tddmm, 'r.-'  , linewidth=2.0, label='Theta_DDMM')
plt.xlabel('Time')
plt.legend()

plt.figure(7)	
plt.plot(pd, 'b.-'  , linewidth=2.0,label='Phi_Dot_BW')
plt.plot(pdm, 'g.-'  , linewidth=2.0,label='Phi_Dot_MED') #[:-1]
plt.xlabel('Time')
plt.legend()

plt.figure(5)	
plt.plot(pdds, 'r.-' , linewidth=2.0, label='Phi_DDotS')
plt.plot(pddm, 'b.-' , linewidth=2.0, label='Phi_DDotM')	
plt.xlabel('Time')
plt.legend()

plt.figure(9)		
plt.plot(pddmm, 'g.-'  , linewidth=2.0, label='Phi_DDotMM')	
plt.xlabel('Time')
plt.legend()


plt.figure(3)		
plt.plot(pddmms, 'g.-'  , linewidth=2.0, label='Phi_DDotMMS')	
plt.xlabel('Time')
plt.legend()

plt.show()








