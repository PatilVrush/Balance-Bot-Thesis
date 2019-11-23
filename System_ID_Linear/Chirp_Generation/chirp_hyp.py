from scipy.signal import chirp, spectrogram
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

w=[];
tf=[]
data = pd.DataFrame() 

fs = 4000
T = 30
t = np.linspace(0, T, T*fs, endpoint=False)
t1 = np.linspace(T, 2*T, T*fs, endpoint=False)

w1 = chirp(t, f0=1, f1=2000, t1=T, method='hyperbolic')

w2 = chirp(t,f0=2000,f1=1,t1=T,method='hyperbolic')

for m in range(len(t)):
	tf.append(t[m])

for n in range(len(t)):
	tf.append(t1[n])


size = len(tf)

for i in range(len(t)):
	w.append((0.5+(0.5*w1[i])))

for j in range(len(t1)):
	w.append((0.5+(0.5*w2[j])))

'''
data = data.assign(K=pd.Series(w).values) 		
data.to_csv("chirp_hyperbolic.csv", index=False)
'''

print(len(w))

plt.figure(3)
plt.plot(tf,w)
plt.title("Hyberbolic Chirp, f(0)=0, f(30)=20000")
plt.xlabel('t (sec)')
plt.show()

plt.figure(4)
ff, tt, Sxx = spectrogram(w, fs=fs, noverlap=256, nperseg=512, nfft=2048)
plt.pcolormesh(tt, ff[:513], Sxx[:513], cmap='gray_r')
plt.title('Quadratic Chirp, f(0)=1500, f(10)=250')
plt.xlabel('t (sec)')
plt.ylabel('Frequency (Hz)')
plt.grid()
plt.show()

