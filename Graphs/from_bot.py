import numpy as np
import matplotlib.pyplot as plt
import csv
from util import FourierSeries as FS
import pandas as pd
plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"
plt.rcParams.update({'font.size': 40})

# MatPlotlib
import matplotlib.pyplot as plt
from matplotlib import pylab

# Scientific libraries
from numpy import arange,array,ones
from scipy import stats


theta = []
sp_t=[]
phi =[]
sp_d = []
ox = []
oy =[]
head=[]
sphead=[]

with open('inner_loop_heading_distance5.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        theta.append(float(row[0]))
        sp_t.append(float(row[2]))
	phi.append(float(row[5]))
	sp_d.append(float(row[4])*0.0401)
	ox.append(float(row[8]))
	oy.append(float(row[9]))
	head.append(float(row[10]))
	sphead.append(float(row[11]))


plt.figure(3)
plt.plot(phi, lw=3, label='Phi')
plt.plot(sp_d, lw=3, label='Set-Point-Phi')
plt.ylabel('Position')
plt.grid()
plt.legend()
plt.show()


plt.figure(4)
plt.plot(ox, oy,lw=3)
plt.xlabel('Odometry_x')
plt.ylabel('Odometry_y')
plt.title('Trajectory taken')
plt.grid()
plt.show()


plt.figure(5)
plt.plot(theta, lw=3, label='Theta')
plt.plot(sp_t, lw=3, label='Set-Point-Theta')
plt.ylabel('Theta (Tilt)')
plt.grid()
plt.legend()
plt.show()

plt.figure(5)
plt.plot(head, lw=3, label='Heading')
plt.plot(sphead, lw=3, label='Set-Point-Heading')
plt.grid()
plt.legend()
plt.show()



'''
plt.figure(1)
plt.plot(current_m1,voltage,'r*' , current_m1, line1 , label='Resistance' , lw=3, ms=14,)
plt.xlabel('Current')
plt.ylabel('Voltage')
plt.title('Voltage vs. Current Motor_1')
plt.text(1, 8, 'Resistance: %f' % slope1)
plt.legend()
plt.grid()
plt.show()

plt.figure(2)
plt.plot(current_m2,voltage,'g*' , current_m2, line2, label='Resistance' , lw=3, ms=14)
plt.xlabel('Current')
plt.ylabel('Voltage')
plt.title('Voltage vs. Current Motor_2')
plt.text(1, 8, 'Resistance: %f' % slope2)
plt.legend()
plt.grid()
plt.show()
'''
