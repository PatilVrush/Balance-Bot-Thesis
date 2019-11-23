import numpy as np
import matplotlib.pyplot as plt
import csv
from util import FourierSeries as FS
import pandas as pd
from scipy.signal import chirp, spectrogram


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

with open('mtd_chirp_q.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        time1.append(float(row[0]))
	enc1.append(int(row[1]))
	time2.append(float(row[2]))
	enc2.append(int(row[3]))
	current1.append(float(row[4]))
	current2.append(float(row[5]))	
	voltage.append(float(row[6]))


time1f = ((np.array(time1)-3724.99585).tolist())

time2f = ((np.array(time2)-3724.996338).tolist())


plt.figure(1)
plt.plot(time1f,voltage , 'g' ,label='time_d' , linewidth=2.0)
plt.xlabel('Time')
plt.ylabel('Voltage(t)')


plt.figure(2)
plt.plot(time1f,current1, 'r' , label='frequency_d' , linewidth=2.0)	
plt.xlabel('Time')
plt.ylabel('Current(t)')

plt.figure(3)
ff, tt, Sxx = spectrogram(current1, fs=8000, noverlap=256, nperseg=512, nfft=2048)
plt.pcolormesh(tt, ff[:513], Sxx[:513], cmap='gray_r')
plt.title('Logarithmic Chirp, f(0)=1500, f(10)=250')
plt.xlabel('t (sec)')
plt.ylabel('Frequency (Hz)')
plt.grid()
plt.show()



