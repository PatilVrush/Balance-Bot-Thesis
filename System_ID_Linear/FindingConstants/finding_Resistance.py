import numpy as np
import matplotlib.pyplot as plt
import csv
from util import FourierSeries as FS
import pandas as pd
from scipy import stats

plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"

current_m1=[]
voltage=[]
current_m2=[]


with open('mt_datafinal_sin_nm.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        current_m1.append(float(row[4]))
        voltage.append(float(row[6]))
	current_m2.append(float(row[5]))	

plt.figure(1)
plt.plot(current_m1,voltage,'--r*' , label='Resistance')
#plt.axis([0,3000,-1,21])
#plt.xticks(xmarks)
#plt.yticks(ymarks_theta)
plt.xlabel('Current')
plt.ylabel('Voltage')
plt.title('Voltage vs. Current')
plt.legend()
plt.grid()
plt.show()
	

slope1, intercept1, r_value1, p_value1, std_err1 = stats.linregress(current_1,voltage)
line1 = slope1*current_m1+intercept1

plt.plot(current_m1,voltage,'o', xi, line1)
pylab.title('Linear Fit with Matplotlib')
ax = plt.gca()
ax.set_axis_bgcolor((0.898, 0.898, 0.898))
fig = plt.gcf()
py.plot_mpl(fig, filename='linear-Fit-with-matplotlib')


