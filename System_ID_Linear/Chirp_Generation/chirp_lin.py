from scipy.signal import chirp, spectrogram
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

w=[];
tf=[]
data = pd.DataFrame() 

fs = 8000
T = 20
t = np.linspace(0, T, T*fs, endpoint=False)

w1 = chirp(t, f0=1, f1=500, t1=T, method='linear')

#w2 = chirp(t,f0=50,f1=1,t1=T,method='linear')

data = data.assign(K=pd.Series(w1).values) 		
data.to_csv("chirp_linear.csv", index=False)

plt.figure(3)
plt.plot(t,w1)
plt.title("Linear Chirp, f(0)=0, f(30)=20000")
plt.xlabel('t (sec)')
plt.show()

plt.figure(4)
ff, tt, Sxx = spectrogram(w1, fs=fs, noverlap=256, nperseg=512, nfft=2048)
plt.pcolormesh(tt, ff[:513], Sxx[:513], cmap='gray_r')
plt.title('Logarithmic Chirp, f(0)=1500, f(10)=250')
plt.xlabel('t (sec)')
plt.ylabel('Frequency (Hz)')
plt.grid()
plt.show()

