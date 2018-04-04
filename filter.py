#from scipy.signal import butter, lfilter, iirfilter, lfilter_zi, filtfilt

WINDOWSIZE = 10 

acc1 = []
acc2 = []
us1 = [0,0,0]
us2 = [0,0,0]
fsr1 = [0,0]
fsr2 = [0,0]

# bandapss filter parameter
a = []
b = []
f_sample = 100 # [Hz] samplig frequency
order = 1

# bandpass filter local array-like data
_us1Data = []
_us2Data = []

# butterworth filter, sampling freq = 100 Hz, order = 1, low freq = 10Hz
def BW_US1(x):
	us1[0] = us1[1]
	us1[1] = us1[2]
	us1[2] = (3.621681514928615665e-3 * x) + (-0.83718165125602272969 * us1[0]) + (1.82269492519630826877 * us1[1])
	return (us1[0] + us1[2]) +2 * us1[1]

# butterworth filter, sampling freq = 100 Hz, order = 1, low freq = 10Hz
def BW_US2(x):
	us2[0] = us2[1]
	us2[1] = us2[2]
	us2[2] = (3.621681514928615665e-3 * x) + (-0.83718165125602272969 * us2[0]) + (1.82269492519630826877 * us2[1])
	return (us2[0] + us2[2]) +2 * us2[1]

# low pass filter
def LP_FSR2(x):
	fsr2[0]=fsr2[1]
	fsr2[1]=(1.826903512279259290e-1 * x) + (0.634619297554414814209 * fsr2[0])
	return (fsr2[0] + fsr2[1])
	


# moving average filter
def MA_ACC1(x):
	if len(acc1) < WINDOWSIZE:
            acc1.append(x)
        else:
            acc1.pop(0)
            acc1.append(x)
	return sum(acc1) / len(acc1)

# moving average filter
def MA_ACC2(x):
	if len(acc2) < WINDOWSIZE:
            acc2.append(x)
        else:
            acc2.pop(0)
            acc2.append(x)
	return sum(acc2) / len(acc2)
"""
def findBWBandpassParameter(low, high):
	'''
	find the parameter a and b and set global parameter a and b
	'''
    
	global a, b, order, f_sample
	low = low / ( f_sample * 0.5 )
	high = high / ( f_sample * 0.5 )
	b, a = butter(order, [low, high], btype='band')

def applyBWBandpassFilter(data):
	'''
	apply butterworth bandpass filter to data stream
	Butterworth bandpass filter parameters must be calculated before applying the filter
	call function findBWBandpassParameter() to find and set the bandpass filter parameters

	return:
		y: filtered data
	'''
	global a, b		
	y, zf = lfilter(b, a, data)
	return y

def BWBP_US1(x):
	if (len(_us1Data)<=WINDOWSIZE):
		_us1Data.append(x)
	
	if(len(_us1Data) > WINDOWSIZE):
		_us1Data.pop()

	yArray = applyBWBandpassFilter(_us1Data)
	y = yArray[-1]
	
	return y

"""

