'''
    Magnetometer_calibration.py
    
    Description: Script used to calibrate magnetometer and determine hard iron offsets
    
    Revision History
    18 Apr 2016 - Created and debugged
    
    Author: Lars Soltmann

    Notes: - Written for Python3
           - Requires Scipy and Numpy
    
    '''

import math
import numpy as np
import scipy.linalg
import scipy.special
from scipy.stats import norm
import time
import spidev
import sys

sys.path.append('/home/pi/Python3_AP_Library')
sys.path.append('/home/pi/Navio2/Python/navio')

import mpu9250
import rcinput
import leds

led=leds.Led()
rcin=rcinput.RCInput()
imu=mpu9250.MPU9250()
imu.initialize()
time.sleep(1)


led.setColor('Yellow')

# SETUP
# Channel used to move through program, based on Assan X8R6
aileron=1
# PWM limit before next task executes
pwm_limit=1700
RCinput=0.0
# Setup magnetometer lists
mag=np.matrix([[0,0,0]])
magM=np.array([0])
# Setup output files
magcal=open('QR_mag_calib.txt', 'w')
magdata=open('QR_mag_data.txt', 'w')
magdata.write('Mx My Mz\n')

exit_flag1=0

# STEP 1 - collect magnetometer data
# Wait for user to move stick past pwm_limit to start data collection
print('To start calibration process move selected stick past PWM limit')
print('until LED turns green.  Rotate vehicle around all axes and then')
print('move the same stick past the PWM limit again to stop data')
print('the data collection process.\n')
while RCinput<pwm_limit:
    RCinput=float(rcin.read(aileron))
    time.sleep(0.02)
print('Collecting magnetometer data ...')
# Start collecting magnetometer data once stick has been moved past PWM limit
led.setColor('Green')
while True:
    m9a, m9g, m9m = imu.getMotion9()
    temp=np.matrix([[m9m[0],m9m[1],m9m[2]]])
    mag=np.concatenate((mag,temp))
    magM=np.append(magM,math.sqrt(m9m[0]**2+m9m[1]**2+m9m[2]**2))
    RCinput=float(rcin.read(aileron))
    # Stop collecting data once the stick has been moved back to neutral and then again past the limit
    if RCinput<1700:
        exit_flag1=1
    if (RCinput>1700 and exit_flag1==1):
        break
    time.sleep(0.01)
print('Data collection stopped. Processing data ...')
led.setColor('Cyan')
# Remove the first two entries, the first entry is zero from the initialization and the next entry has sometimes been much larger or smaller in magnitude than the rest
mag=np.delete(mag,0,0)
mag=np.delete(mag,0,0)
magM=np.delete(magM,0,0)
magM=np.delete(magM,0,0)

# STEP 2 - process magnetometer data to determine hard-iron offsets
# Remove any outliers
magM_m=np.mean(magM)
magM_sd=np.std(magM)
N=np.size(magM)

DEV=2.5

# Maximum allowable deviation 
Dmax_xu=DEV*magM_sd+magM_m
Dmax_xl=-DEV*magM_sd+magM_m

remove_count=0
for x in range(N):
    if (magM[x]>Dmax_xu or magM[x]<Dmax_xl):
        mag=np.delete(mag,x,0)
        remove_count=remove_count+1

print('Number of outliers removed = %d' % remove_count)

d1=np.multiply(mag[:,0],mag[:,0])
d2=np.multiply(mag[:,1],mag[:,1])
d3=np.multiply(mag[:,2],mag[:,2])
d4=2*np.multiply(mag[:,0],mag[:,1])
d5=2*np.multiply(mag[:,0],mag[:,2])
d6=2*np.multiply(mag[:,1],mag[:,2])
d7=2*mag[:,0]
d8=2*mag[:,1]
d9=2*mag[:,2]

D=np.concatenate((d1,d2,d3,d4,d5,d6,d7,d8,d9),1)

d10=np.transpose(D)*np.asmatrix(np.ones((np.size(D[:,1]),1)))
d11=np.transpose(D)*D

v=scipy.linalg.solve(d11,d10)

A=np.matrix([[v.item(0),v.item(3),v.item(4),v.item(6)],[v.item(3),v.item(1),v.item(5),v.item(7)],[v.item(4),v.item(5),v.item(2),v.item(8)],[v.item(6),v.item(7),v.item(8),-1]])
v1=np.asmatrix(np.array([[v.item(6)],[v.item(7)],[v.item(8)]]))

center=scipy.linalg.solve(-A[0:3,0:3],v1)

T=np.eye(4)
T[3,0:3]=np.transpose(center)
T=np.asmatrix(T)

R=T*A*np.transpose(T)

R1=R[0:3,0:3]
R2=-R[3,3]

eigval,eigvec=scipy.linalg.eig(R1/R2)
'''
if (eigval.item(0).imag != 0 or eigval.item(1).imag != 0 or eigval.item(2).imag != 0):
    sys.exit('Radii are imaginary!')

if (eigval.item(0).real<0 or eigval.item(1).real<0 or eigval.item(2).real<0):
    sys.exit('Radii are negative!')

print(eigval)

rad_x=math.sqrt(1/eigval.item(0).real)
rad_y=math.sqrt(1/eigval.item(1).real)
rad_z=math.sqrt(1/eigval.item(2).real)
'''
print('Hard iron offsets = %f.2 %f.2 %f.2\n' % (center.item(0),center.item(1),center.item(2)))

print('Writing data to files ...')
#magcal.write('MAGNETOMETER CALIBRATION FILE\n')
#magcal.write('Hard iron offsets\n')
magcal.write('%.3f %.3f %.3f\n' % (center.item(0),center.item(1),center.item(2)))
#magcal.write('%.3f %.3f %.3f\n' % (eigval.item(0).real,eigval.item(1).real,eigval.item(2).real))
#magcal.write('Radii\n')
#magcal.write('%.3f %.3f %.3f\n' % (rad_x,rad_y,rad_z))

for x in range(np.size(mag[:,0])):
    magdata.write('%.3f %.3f %.3f\n' % (mag[x,0],mag[x,1],mag[x,2]))

magcal.close()
magdata.close()
led.setColor('Black')
print('Done.')
