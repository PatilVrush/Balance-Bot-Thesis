from scipy.signal import chirp, spectrogram
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


data = pd.DataFrame() 

fs = 8000
T = 30
w=[]
t = np.linspace(0, T, T*fs, endpoint=False)
w1 = chirp(t, f0=1, f1=200, t1=T, method='quadratic')

for i in range(len(t)):
	w.append((0.5+(0.5*w1[i])))


'''
data = data.assign(K=pd.Series(w).values) 		
data.to_csv("chirp_quadratic.csv", index=False)
'''

print(len(w))	
plt.figure(3)
plt.plot(t,w)
plt.title("Quadratic Chirp, f(0)=0, f(30)=200	00")
plt.xlabel('t (sec)')
plt.show()

plt.figure(4)
ff, tt, Sxx = spectrogram(w1, fs=fs, noverlap=256, nperseg=512, nfft=2048)
plt.pcolormesh(tt, ff[:513], Sxx[:513], cmap='gray_r')
plt.title('Quadratic Chirp, f(0)=1500, f(10)=250')
plt.xlabel('t (sec)')
plt.ylabel('Frequency (Hz)')
plt.grid()
plt.show()

