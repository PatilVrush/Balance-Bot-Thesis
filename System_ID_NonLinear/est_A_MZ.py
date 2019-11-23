import numpy as np
import sklearn
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt
import csv
from util import FourierSeries as FS
import pandas as pd
import scipy.signal as SI
import numpy.linalg as la
from util import *
from scipy.odr import *
import random

#plt.rcParams["font.weight"] = "bold"
#plt.rcParams["axes.l	abelweight"] = "bold"
plt.rcParams.update({'font.size': 40})

theta=[]
theta_dot=[]
phi=[]
phi_dot=[]
time=[]
tdd=[]
pdd=[]

with open('FT_ME_P5_S1.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
		theta.append(float(row[0]))
		theta_dot.append(float(row[6]))
		phi.append(float(row[2]))
		phi_dot.append(float(row[7]))
		tdd.append(float(row[10]))
		pdd.append(float(row[11]))
	
matrix1 = np.column_stack((theta,theta_dot,phi,phi_dot))

'''
def linear_func(p, x):
   m = p
   return m*x 

# Create a model for fitting.
linear_model = Model(linear_func)

# Create a RealData object using our initiated data from above.
data = RealData(matrix1, tdd)

# Set up ODR with the model and data.
odr = ODR(data, linear_model, beta0=[0.])

# Run the regression.
out = odr.run()

# Use the in-built pprint method to give us results.
out.pprint()
'''

print('\n******************************* FT_ME_P1_S1_Z.csv NS ******************\n')
print('\n\n*************************Total Least Squares Using SVD***********************\n\n')

#Total Least Squares Using SVD

def tls(X,y):
    
    if X.ndim is 1: 
        n = 1 # the number of variable of X
        X = X.reshape(len(X),1)
    else:
        n = np.array(X).shape[1] 
    
    Z = np.vstack((X.T,y)).T
    U, s, Vt = la.svd(Z, full_matrices=True)
    
    V = Vt.T
    Vxy = V[:n,n:]
    Vyy = V[n:,n:]
    a_tls = - Vxy  / Vyy # total least squares soln
    
    Xtyt = - Z.dot(V[:,n:]).dot(V[:,n:].T)
    Xt = Xtyt[:,:n] # X error
    y_tls = (X+Xt).dot(a_tls)
    fro_norm = la.norm(Xtyt, 'fro')
    
    return y_tls, X+Xt, a_tls, fro_norm

te,met,mt,ft = tls(matrix1,tdd)

print(mt)

#tdde = np.matmul(matrix1, mt)

pe,mep,mp,fp = tls(matrix1,pdd)

print(mp)

#pdde = np.matmul(matrix1, mp)


print('\n\n******************************PRINCOMP************************\n\n')

matrixt = np.column_stack((theta,theta_dot,phi,phi_dot,tdd))
matrixt_m = matrixt - mean(matrixt,0)[newaxis,:]
matrixp = np.column_stack((theta,theta_dot,phi,phi_dot,pdd))
matrixt_p = matrixp - mean(matrixp,0)[newaxis,:]
#PCA

Lt,Vt,St = princomp(matrixt)
ct = -Vt[0:4,4]/Vt[4,4]
print(ct)
tdde = np.matmul(matrix1, ct)

Lp,Vp,Sp = princomp(matrixp)
cp = -Vp[0:4,4]/Vp[4,4]
print(cp)

pdde = np.matmul(matrix1, cp)


#Ordinary Least Squares
print('\n*****************Ordinary Least Squares************************\n')

A_plus1 = np.linalg.pinv(matrix1)
print(A_plus1.shape)

res1 = A_plus1.dot(tdd)
print(res1)

tdde2 = np.matmul(matrix1, res1)

res2 = A_plus1.dot(pdd)
print(res2)

pdde2 = np.matmul(matrix1, res2)

res3 = A_plus1.dot(theta_dot)
print(res3)

res4 = A_plus1.dot(phi_dot)
print(res4)

#Figures
'''
plt.figure(4)	
plt.plot(time,tdd, 'b.-'  , linewidth=4.0 , label='TDD')
plt.plot(time,tdde, 'r.-'  , linewidth=2.0 ,label='TDD_TLE')
plt.plot(time,tdde2, 'g.-'  , linewidth=2.0, label='TDD_OLS')
plt.xlabel('Time')
plt.legend()


plt.figure(7)	
plt.plot(time,pdd, 'b.-'  , linewidth=2.0,label='PDD')
plt.plot(time,pdde, 'r.-'  , linewidth=2.0,label='PDD_TLS')
plt.plot(time,pdde2, 'g.-'  , linewidth=2.0, label='PDD_OLS') #[:-1]
plt.xlabel('Time')
plt.legend()

'''
plt.figure(6)	
plt.plot(tdd, 'b.-'  , linewidth=2.0, label='TDD')
plt.plot(theta, 'g.-'  , linewidth=2.0, label='TDD_E2')
plt.xlabel('Time')
plt.legend()

plt.figure(5)	
plt.plot(pdd, 'r.-' , linewidth=2.0, label='PDD')	
plt.plot(phi_dot, 'g.-'  , linewidth=2.0, label='PDD_E2')	
plt.xlabel('Time')
plt.legend()

plt.show()

