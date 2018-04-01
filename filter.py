
WINDOWSIZE = 10 

acc1 = []
acc2 = []
us1 = [0,0,0]
us2 = [0,0,0]

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
	us2[2] = (3.621681514928615665e-3 * x) + (-0.83718165125602272969 * us1[0]) + (1.82269492519630826877 * us2[1])
	return (us2[0] + us2[2]) +2 * us2[1]

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

