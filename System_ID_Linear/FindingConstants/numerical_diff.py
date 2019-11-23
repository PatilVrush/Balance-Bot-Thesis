from numpy import *
from scipy.signal import butter,filtfilt,correlate
 
def mov_avg2(x):
    y = zeros(len(x)-1)
    for i in xrange(len(x)-1):
    	y[i] = .5*(x[i]+x[i+1])
    return y

def d_smooth(x,t):
	b0,a0 = butter(2,0.999) # 0.2
	if (x.ndim == 1):
		d = 1
		xdot = zeros((len(t)-2,d))
		# xf1  = filtfilt(b0,a0,x)
		dxf1  = diff(x)/diff(t)
		# dxf2 = filtfilt(b0,a0,dxf1)
		xdot = mov_avg2(dxf1)
	else:
		d = x.shape[1]
		xdot = zeros((len(t)-2,d))
		for i in xrange(d):
			# xf1  = filtfilt(b0,a0,x[:,i])
			dxf1  = diff(x[:,i])/diff(t)
			# dxf2 = filtfilt(b0,a0,dxf1)
			xdot[:,i] = mov_avg2(dxf1)
	return xdot

def d_smooth_a(x,t,a):
	b0,a0 = butter(a[0],a[1]) # 0.2
	if (x.ndim == 1):
		d = 1
		xdot = zeros((len(t)-2,d))
		xf1  = filtfilt(b0,a0,x)
		dxf1  = diff(xf1)/diff(t)
		dxf2 = filtfilt(b0,a0,dxf1)
		xdot = mov_avg2(dxf2)
	else:
		d = x.shape[1]
		xdot = zeros((len(t)-2,d))
		for i in xrange(d):
			xf1  = filtfilt(b0,a0,x[:,i])
			dxf1  = diff(xf1)/diff(t)
			dxf2 = filtfilt(b0,a0,dxf1)
			xdot[:,i] = mov_avg2(dxf2)
	return xdot


def smooth(x,t):
	b0,a0 = butter(2,.8) # 0.2
	xf1  = filtfilt(b0,a0,x)
	return xf1

def lowpass(x,b):
	b0,a0 = butter(2,b)
	xf1  = filtfilt(b0,a0,x)
	return xf1

def trim(x,a):
	if (a > len(x)): return x[:,a:-a]
	return x[a:-a]
