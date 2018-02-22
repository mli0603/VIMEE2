WINDOWSIZE = 50

# convert digital data to acceleration in unit of g
def toDegree(radian):
    return radian/3.1415926*180

# convert ultrasound duration to cm
def toCm(duration):
    return (duration/2) / 29.1

