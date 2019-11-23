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

with open('mtd_chirp_h_ws.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        time1.append(float(row[0]))
	enc1.append(int(row[1]))
	time2.append(float(row[2]))
	enc2.append(int(row[3]))
	current1.append(float(row[4]))
	current2.append(float(row[5]))	
	voltage.append(float(row[6]))


time1f = ((np.array(time1)-240.855362).tolist())

time2f = ((np.array(time2)-240.855759).tolist())

l = (len(enc1))
for i in range(l):
	theta1.append((enc1[i]/979.2)*2*3.14)

plt.figure(1)
#plt.subplot(211)
plt.plot(time1f,current1 , 'g' ,label='time_d' , linewidth=1.0)
#plt.subplot(212)
#plt.plot(time1f,theta1 , 'r' ,label='time_d' , linewidth=1.0)
plt.xlabel('Time')
plt.ylabel('Current')



plt.figure(2)
plt.plot(time1f,voltage, 'r' , label='frequency_d' , linewidth=2.0)	
plt.xlabel('Time')
plt.ylabel('Voltage')
plt.show()

plt.figure(4)
ff, tt, Sxx = spectrogram(current1, fs=4000, noverlap=128, nperseg=512, nfft=2048)
plt.pcolormesh(tt, ff[:513], Sxx[:513], cmap='gray_r')
plt.title('Logarithmic Chirp, f(0)=1500, f(10)=250')
plt.xlabel('t (sec)')
plt.ylabel('Frequency (Hz)')
plt.grid()
plt.show()
	


