'''
    QR_manual_althold.py
    
    Description: Manual quadrotor control with altitude recording and hold
    
    Revision History
    17 Mar 2016 - Created and debugged
    17 Apr 2016 - Replaced threading module with multiprocessing module because python GIL has problems with multithreading   
    28 Apr 2016 - Replaced yaw rate controller with heading hold controller
    04 May 2016 - Added altitude hold option
 
    Author: Lars Soltmann
    
    Notes:
    Written for Python3 using the RPi3+NAVIO2 by Emlid
    
    *RC INPUT
    Ch1 - Roll       [0]
    Ch2 - Pitch      [1]
    Ch3 - Throttle   [2]
    Ch4 - Yaw        [3]
    Ch5 - Mode       [4]
    
    *MOTOR NUMBERING AND AXIS SYSTEM
    
    Forward
      ^
      |
    1   2     x
     \ /      |
      |       0-y
     / \     /
    4   3   z (into page)
    
    1 & 3 = CW
    2 & 4 = CCW
    
    Aircraft body-axis coordinate system, right hand rule
    
    LED Guide:
    Yellow = Initialization - do not move vehicle
    Green = Ready to fly, 3D GPS fix
    Cyan = Ready to fly, no GPS fix
    Red (flashing) = throttle not set at zero, move throttle to zero
    Red (solid) = Error
    
    
    '''

from multiprocessing import Process, Array, Value
import time
import sys
import os

sys.path.append('/home/pi/Python3_AP_Library')
sys.path.append('/home/pi/Navio2/Python/navio')

import mpu9250
import Complementary_Filter2
import Read_Config
import PID
import rcinput
import pwm
import leds
import MB1242
import ms5611

# Exit_flag=0 during normal run and 1 when ready to exit
exit_flag=0
# Used to capture toggle time of gear swtich to exit program
prev_tgear=0
tgear=0
gearflag=0

dt=0 #Initialization at zero for heading hold

# Initialization of lists
# Motor commands - [M1,M2,M3,M4]
motor=[0,0,0,0]
# PID control outputs - [roll,pitch,throttle,yaw]
delta_control=[0,0,0,0]
# Target angle and rate values - [pitch,roll,yaw]
targets=[0,0,0]
# RCinput from receiver - [pitch,roll,yaw,throttle]
RCinput=[0,0,0,0,0]


def attitude(processEXIT,output_array):
    att=Complementary_Filter2.comp_filt(output_array[6],output_array[7],output_array[8])
    imu=mpu9250.MPU9250()
    imu.initialize()
    time.sleep(1)
    g_offset=[0,0,0]

    # Low pass filter setup
    fc=15; #Hz
    dt_f=0.0011 #measured average (sec)
    pie=3.14159265
    a_f=2*pie*dt_f*fc/(2*pie*dt_f*fc+1)
    b_f=1-a_f
    first_time=1

    # Loop to determine gyroscope offsets
    for x in range(0, 100):
        m9a, m9g, m9m = imu.getMotion9()
        g_offset[0]=g_offset[0]+m9g[1]/100
        g_offset[1]=g_offset[1]+m9g[0]/100
        g_offset[2]=g_offset[2]+m9g[2]/100    
    g_offset[2]=-g_offset[2] #because z-axis is oriented 180deg

    if (m9m[0]==0.0 or m9m[1]==0.0 or m9m[2]==0.0):
        output_array[9]=1.0

    while processEXIT.value==0:
        m9a, m9g, m9m = imu.getMotion9()
        gx=(m9g[1]-g_offset[0])*57.2958 #Convert to deg/s
        gy=(m9g[0]-g_offset[1])*57.2958
        gz=(-m9g[2]-g_offset[2])*57.2958
        ax=m9a[1]*0.10197 #Convert to g-force (1/9.81)
        ay=m9a[0]*0.10197
        az=-m9a[2]*0.10197
        att.attitude3(ax,ay,az,gx,gy,gz,m9m[0],m9m[1],m9m[2])

        # Apply LPF (only for ouput data)
        if first_time==1:
            theta_dot_d=att.thetad_d
            phi_dot_d=att.phid_d
            psi_dot_d=att.psid_d
            first_time=0
        else:
            theta_dot_d=a_f*att.thetad_d+b_f*theta_dot_d
            phi_dot_d=a_f*att.phid_d+b_f*phi_dot_d
            psi_dot_d=a_f*att.psid_d+b_f*psi_dot_d
     
        # Set outputs
        output_array[0]=att.roll_d-output_array[10]  #Subtract offsets due to orientation error
        output_array[1]=att.pitch_d-output_array[11] #Subtract offsets due to orientation error
        output_array[2]=att.yaw_d
        output_array[3]=psi_dot_d
        output_array[4]=phi_dot_d
        output_array[5]=theta_dot_d
    
    return None


def altitude(processEXIT,output_array,AHRS_array):
    #Range finder initialization
    RF=MB1242.MB1242(0x70)
    rf_mat=[0.0]*15
    
    #Barometer initialization
    baro=ms5611.MS5611()
    baro.initialize()
    APRES_B=1013.25
    first_time=1
    previous_rf=0

    #Kalman filter initialization
    h1=1
    h2=1
    p1=1
    p2=0
    p3=1
    p4=0

    q1=0.1
    q2=3.0
    r1=0.1
    r2=0.31

    x1=0
    x2=0

    tstart=time.time()
    while processEXIT.value==0:
        t1=time.time()
        #Get barometer and range finder data
        RF.refreshDistance()
        baro.refreshPressure()
        time.sleep(0.01)
        baro.readPressure()
        baro.refreshTemperature()
        time.sleep(0.01)
        baro.readTemperature()
        baro.calculatePressureAndTemperature()
        time.sleep(0.08)
        RF.readDistance()

        if first_time==1:
            pavg_ground=baro.PRES
            first_time=0
            rf_raw=0
            baro_raw=0
        else:
            rf_raw=RF.dist*0.0328084
            baro_raw=-27.4448*(baro.PRES-pavg_ground)

        t2=time.time()
        dt=t2-t1
        t_elapsed=t2-tstart

        #Kalman filter for altitude
        #range finder = 1, barometer = 2
        z1=rf_raw
        z2=baro_raw

        #Don't use range finder data if vehicle is above 25ft or too close to ground
        if ((rf_raw>25) or abs(rf_raw-previous_rf)>1):
            h1=0
        else:
            h1=1
        #Create simplifications to reduce multiplication calls
        ksim1=(h1*p1*(h2**2*p1+r2)/(h1**2*p1*r2+r1*h2**2*p1+r1*r2)
              -h2**2*p1**2*h1/(h1**2*p1*r2+r1*h2**2*p1+r1*r2))
        ksim2=(-h1**2*p1**2*h2/(h1**2*p1*r2+r1*h2**2*p1+r1*r2)
              +h2*p1*(h1**2*p1+r1)/(h1**2*p1*r2+r1*h2**2*p1+r1*r2))
        ksim3=(p3*h1*(h2**2*p1+r2)/(h1**2*p1*r2+r1*h2**2*p1+r1*r2)
              -p3*h2**2*h1*p1/(h1**2*p1*r2+r1*h2**2*p1+r1*r2))
        ksim4=(-p3*h1**2*p1*h2/(h1**2*p1*r2+r1*h2**2*p1+r1*r2)
              +p3*h2*(h1**2*p1+r1)/(h1**2*p1*r2+r1*h2**2*p1+r1*r2))
        #Correct state estimate
        xest1=x1+ksim1*(z1-h1*x1)+ksim2*(z2-h2*x1)                 
        xest2=x2+ksim3*(z1-h1*x1)+ksim4*(z2-h2*x1)
        #Corect covariance estimates
        pest1=(1-ksim1*h1-ksim2*h2)*p1
        pest2=(1-ksim1*h1-ksim2*h2)*p2
        pest3=(-ksim3*h1-ksim4*h2)*p1+p3
        pest4=(-ksim3*h1-ksim4*h2)*p2+p4
        #Predict next state
        xpred1=x1+ksim1*(z1-h1*x1)+ksim2*(z2-h2*x1)+dt*(x2+ksim3*(z1-h1*x1)+ksim4*(z2-h2*x1))
        xpred2=x2+ksim3*(z1-h1*x1)+ksim4*(z2-h2*x1)
        #Predict next covariance matrix
        ppred1=((1-ksim1*h1-ksim2*h2)*p1+dt*((-ksim3*h1-ksim4*h2)*p1+p3)
               +((1-ksim1*h1-ksim2*h2)*p2+dt*((-ksim3*h1-ksim4*h2)*p2+p4))*dt+q1)
        ppred2=(1-ksim1*h1-ksim2*h2)*p2+dt*((-ksim3*h1-ksim4*h2)*p2+p4)
        ppred3=(-ksim3*h1-ksim4*h2)*p1+p3+dt*((-ksim3*h1-ksim4*h2)*p2+p4)
        ppred4=(-ksim3*h1-ksim4*h2)*p2+p4+q2
        #Assign variables for the next loop
        x1=xpred1
        x2=xpred2
        p1=ppred1
        p2=ppred2
        p3=ppred3
        p4=ppred4
        previous_rf=rf_raw

        #Set outputs
        output_array[0]=xest1
        output_array[1]=xest2

    return None


def RCinput_to_angles(RCinput,RC_T_A_constants,target_psi):
    #                       0               1         2         3           4         5           6        7      8           9         10         11         12
    #RC_T_A_constants=[config.dead_band,config.Pcn,config.Rcn,config.Ycn,config.Pm,config.Pbl,config.Pbh,self.Rm,config.Rbl,config.Rbh,config.Ym,config.Ybl,config.Ybh]
    R=RCinput[0]
    P=RCinput[1]
    Y=RCinput[3]
    
    #PITCH - deg
    if P>RC_T_A_constants[0]+RC_T_A_constants[1]:
        targets[1]=-RC_T_A_constants[4]*P-RC_T_A_constants[6]
    elif P<RC_T_A_constants[1]-RC_T_A_constants[0]:
        targets[1]=-RC_T_A_constants[4]*P-RC_T_A_constants[5]
    else:
        targets[1]=0
    
    #ROLL - deg
    if R>RC_T_A_constants[0]+RC_T_A_constants[2]:
        targets[0]=-RC_T_A_constants[7]*R-RC_T_A_constants[9]
    elif R<RC_T_A_constants[2]-RC_T_A_constants[0]:
        targets[0]=-RC_T_A_constants[7]*R-RC_T_A_constants[8]
    else:
        targets[0]=0

    #YAW - deg
    if Y>RC_T_A_constants[0]+RC_T_A_constants[3]:
        target_yaw_rate=-RC_T_A_constants[10]*Y-RC_T_A_constants[12]
        targets[2]=target_psi+target_yaw_rate*dt
        if targets[2]>360:
            targets[2]=targets[2]-360
        if targets[2]<0:
            targets[2]=targets[2]+360
    elif Y<RC_T_A_constants[3]-RC_T_A_constants[0]:
        target_yaw_rate=-RC_T_A_constants[10]*Y-RC_T_A_constants[11]
        targets[2]=target_psi+target_yaw_rate*dt
        if targets[2]>360:
            targets[2]=targets[2]-360
        if targets[2]<0:
            targets[2]=targets[2]+360
    else:
        targets[2]=target_psi

    return targets

def MOTOR_COMMANDS(delta_control,T_cut,throttle):
    R=delta_control[0]
    P=delta_control[1]
    T=delta_control[2]
    Y=delta_control[3]
    motor[0]=T+P*0.5+R*0.5-Y*0.5 #pwm, u_sec
    motor[1]=T+P*0.5-R*0.5+Y*0.5 #pwm, u_sec
    motor[2]=T-P*0.5-R*0.5-Y*0.5 #pwm, u_sec
    motor[3]=T-P*0.5+R*0.5+Y*0.5 #pwm, u_sec
    
    #Limit PWM output range
    if motor[0]>config.PWM_MAX:
        motor[0]=config.PWM_MAX
    elif motor[0]<config.PWM_MIN:
        motor[0]=config.PWM_MIN

    if motor[1]>config.PWM_MAX:
        motor[1]=config.PWM_MAX
    elif motor[1]<config.PWM_MIN:
        motor[1]=config.PWM_MIN

    if motor[2]>config.PWM_MAX:
        motor[2]=config.PWM_MAX
    elif motor[2]<config.PWM_MIN:
        motor[2]=config.PWM_MIN

    if motor[3]>config.PWM_MAX:
        motor[3]=config.PWM_MAX
    elif motor[3]<config.PWM_MIN:
        motor[3]=config.PWM_MIN
  
    # If throttle is below cut-off value then turn all motors off
    if throttle<T_cut:
        motor[0]=config.PWM_MIN
        motor[1]=config.PWM_MIN
        motor[2]=config.PWM_MIN
        motor[3]=config.PWM_MIN

    return motor

def check_CLI_inputs():
    #Modes:
    # 1 = Normal operation mode
    # 2 = ESC calibration, straight pass through from RCINPUT
    # 3 = Control calibration mode, map RCINPUT to target angles
    # 4 = Normal operation mode with data logging

    if len(sys.argv)==1:
        mode=1
        print('No command line inputs found ... entering normal operation mode.')
    elif len(sys.argv)>1:
        if sys.argv[1]=='1':
            mode=1
            print('Entering normal operation.')
        elif sys.argv[1]=='2':
            mode=2
            print('Entering ESC calibration mode.')
        elif sys.argv[1]=='3':
            mode=3
            print('Entering control calibration mode.')
        elif sys.argv[1]=='4':
            mode=4
            print('Entering normal operation with data logging.')
        else:
            sys.exit('Unknown input argument!')
    return mode



##### MAIN PROGRAM #####
# Read command line inputs during program excecution and direct program accordingly
mode=check_CLI_inputs()

# Setup LED
led=leds.Led()
led.setColor('Yellow')

# Read user configuration file
config=Read_Config.read_config_file('QR_config.txt','QR_calib.txt','QR_mag_calib.txt')
config.read_configuration_file()
if (mode==1 or mode==4):
    config.read_calibration_file()
    config.calibration_check()
    config.read_magnetometer_calibration_file()

# Setup RCinput and RCoutput
rcin=rcinput.RCInput()
rcou1=pwm.PWM(0)
rcou2=pwm.PWM(1)
rcou3=pwm.PWM(2)
rcou4=pwm.PWM(3)
rcou1.set_period(config.PWM_FREQ) #Hz
rcou2.set_period(config.PWM_FREQ)
rcou3.set_period(config.PWM_FREQ)
rcou4.set_period(config.PWM_FREQ)

# NORMAL OPERATION
if (mode==1 or mode==4):
    # Constants for RCinput to angle equations
    RC_T_A_constants=[config.dead_band,config.Pcn,config.Rcn,config.Ycn,config.Pm,config.Pbl,config.Pbh,config.Rm,config.Rbl,config.Rbh,config.Ym,config.Ybl,config.Ybh]
    
    # Initialize PID controllers
    # for definitions of variables ... see read_config.py
    pid_pitch=PID.PID(config.p_pitch,config.d_pitch,config.i_pitch,config.il_pitch)
    pid_roll=PID.PID(config.p_roll,config.d_roll,config.i_roll,config.il_roll)
    pid_yaw=PID.PID(config.p_yaw,config.d_yaw,config.i_yaw,config.il_yaw)
    pid_alt=PID.PID(config.p_alt,config.d_alt,config.i_alt,config.il_alt)

    # Start attitude estimation subprocess
    # AHRS_data array format
    # [0]=AHRS roll angle (deg)
    # [1]=ARHS pitch angle (deg)
    # [2]=AHRS yaw angle (deg)
    # [3]=AHRS psi dot (deg/s)
    # [4]=AHRS phi dot (deg/s)
    # [5]=AHRS theta dot (deg/s)
    # [6]=hard iron offest x
    # [7]=hard iron offset y
    # [8]=hard iron offset z
    # [9]=magnetometer function check
    AHRS_data=Array('d', [0.0,0.0,0.0,0.0,0.0,0.0,config.hix,config.hiy,config.hiz,0.0,config.sys_offset_x,config.sys_offset_y])
    # ALT_data array format
    # [0]=altitude (ft)
    # [1]=rate of altitude change (ft/s)
    ALT_data=Array('d',[0.0,0.0])
    process_EXIT=Value('i', 0)
    AHRS_proc=Process(target=attitude, args=(process_EXIT,AHRS_data))
    ALT_proc=Process(target=altitude, args=(process_EXIT,ALT_data,AHRS_data))
    AHRS_proc.start()
    ALT_proc.start()
    time.sleep(3)

    # Maximum throttle setting for which all motors remain off
    T_cut=config.Tmin+config.thr_cut

    # Check to make sure magnetometer is functioning correctly and not reporting all zeros
    if AHRS_data[9]==1.0:
        led.setColor('Red')
        exit_flag=1
        process_EXIT.value=1
        AHRS_proc.join()
        sys.exit('Magnetometer reading zero!')

    # Warn user if throttle is not set to zero during startup
    while (float(rcin.read(0))>T_cut):
        led.setColor('Red')
        time.sleep(0.25)
        led.setColor('Black')
        time.sleep(0.25)

    # Setup up data log if mode=4
    if mode==4:
        print('Setting up flight log.')
        flt_log=open('flight_log.txt', 'w')
        flt_log.write('t dt phi theta psi phi_dot theta_dot psi_dot m1 m2 m3 m4 alt d_alt\n')

    led.setColor('Cyan')
    print('Initialization complete. Starting control loops...')    
    t_start=time.time()

    count=0 # Used for reduced frame rate output to screen

    # Set the initial yaw target as the current one
    targets[2]=AHRS_data[2]
    
    # Altitude hold flag
    alt_hold_flag=0

    while exit_flag==0:
        t_1=time.time()

        # Definition is based of Assan X8R6
        RCinput[0]=float(rcin.read(1)) #Roll
        RCinput[1]=float(rcin.read(2)) #Pitch
        RCinput[2]=float(rcin.read(0)) #Throttle
        RCinput[3]=float(rcin.read(3)) #Yaw
        RCinput[4]=float(rcin.read(4)) #Mode

        # Reset heading hold to current heading whenever throttle goes below t_cut
        if  RCinput[2]<T_cut:
            targets[2]=AHRS_data[2]

        # Set the altitude hold with MODE switch and set it to the current altitude
        if (RCinput[4]<1500) and (alt_hold_flag==0):
            ALT_target=ALT_data[0]
            THR_input_at_hold=RCinput[2]
            alt_hold_flag=1
        if RCinput[4]>1500:
            alt_hold_flag=0

        # Convert receiver input to angles and rate for yaw
        targets=RCinput_to_angles(RCinput,RC_T_A_constants,targets[2])

        # Calculate control inputs
        # order of inputs: target, actual, rate
        delta_control[0]=pid_roll.control2(targets[0],AHRS_data[0],AHRS_data[4])
        delta_control[1]=pid_pitch.control2(targets[1],AHRS_data[1],AHRS_data[5])
        delta_control[3]=pid_yaw.control2(targets[2],AHRS_data[2],AHRS_data[3])
        if RCinput[4]<1500:
            delta_control[2]=pid_alt.control2(ALT_target,ALT_data[0],ALT_data[1])+THR_input_at_hold
        else:
            delta_control[2]=RCinput[2]      

        # Calculate motor commands
        motor=MOTOR_COMMANDS(delta_control,T_cut,RCinput[2])

        # Send motor commands to RCoutput
        rcou1.set_duty_cycle(motor[0]*0.001) # u_sec to m_sec
        rcou2.set_duty_cycle(motor[1]*0.001)
        rcou3.set_duty_cycle(motor[2]*0.001)
        rcou4.set_duty_cycle(motor[3]*0.001)

        # Determine if gear switch has been toggled rapidly (less than 0.5sec
        # between ON and ON) to exit program
        prev_tgear=tgear
        if RCinput[4]<1500:
            if gearflag==0:
                tgear=time.time()
                gearflag=1
        else:
            gearflag=0
        if ((tgear-prev_tgear)<0.5 and (tgear-prev_tgear)>0) and prev_tgear != 0:
            exit_flag=1
            process_EXIT.value=1

        t_2=time.time()
        dt=t_2-t_1

        if mode==4:
            t_elapsed=t_2-t_start
            flt_log.write('%.3f %.4f %.2f %.2f %.2f %.2f %.2f %.2f %d %d %d %d %.2f %.2f\n' % (t_elapsed,dt,AHRS_data[0],AHRS_data[1],AHRS_data[2],AHRS_data[4],AHRS_data[5],AHRS_data[3],int(motor[0]),int(motor[1]),int(motor[2]),int(motor[3]),ALT_data[0],ALT_data[1]))

        #DEBUG - output to screen
        #count=count+1
        #if count==200:
        #    print('%.2f %.2f %.2f %.1f' % (targets[0],targets[1],targets[2],1/dt))
        #    print('%.2f %.2f %.2f %.1f' % (AHRS_data[0],AHRS_data[1],AHRS_data[2],1/dt))
        #    count=0









#------------------------------------- CALIBRATION SCRIPTS --------------------------------

# ESC CALIBRATION MODE
# in this mode all channels are slaved to throttle channel directly with PWM limiting
if mode==2:
    led.setColor('Black')
    print('ESC CALIBRATION MODE')
    print(' ')
    dummy=input('Press any key to start')
    print(' ')
    while exit_flag==0:
        # Get inputs from the receiver and slave them to throttle channel
        RCinput[0]=float(rcin.read(0)) #Throttle
        RCinput[4]=float(rcin.read(4)) #Mode
        # Limit PWM output between MIN MAX values
        if RCinput[0]>config.PWM_MAX:
            motor[0]=config.PWM_MAX
        elif RCinput[0]<config.PWM_MIN:
            motor[0]=config.PWM_MIN
        else:
            motor[0]=RCinput[0]
            
        # Send motor commands to RCoutput
        rcou1.set_duty_cycle(motor[0]*0.001) # u_sec to m_sec
        rcou2.set_duty_cycle(motor[0]*0.001)
        rcou3.set_duty_cycle(motor[0]*0.001)
        rcou4.set_duty_cycle(motor[0]*0.001)

        # Output to console
        print('PWM out = %d uS' % motor[0])
        time.sleep(0.02)

        # Determine if gear switch has been toggled rapidly (less than 0.5sec
        # between ON and ON) to exit program
        prev_tgear=tgear
        if RCinput[4]<1500:
            if gearflag==0:
                tgear=time.time()
                gearflag=1
        else:
            gearflag=0
        if ((tgear-prev_tgear)<0.5 and (tgear-prev_tgear)>0) and prev_tgear != 0:
            exit_flag=1

# CONTROL CALIBRATION MODE
# in this mode the PWM to pitch/roll/yaw_rate curves are determined based on the user input file
if mode==3:
    led.setColor('Black')
    print('CONTROL CALIBRATION MODE')
    print(' ')
    print('Step 1: Determine center points.')
    print('        Do not move sticks for 5 seconds.')
    print(' ')
    dummy=input('Press RETURN to start ')
    print(' ')
    print('Determining center points...')
    print(' ')
    CENTER_PWM=[0,0,0,0]
    t1=time.time()
    x=0.0
    i=1
    while x<5.0:
        # Get inputs from the receiver
        # Definition is based of Assan X8R6
        RCinput[0]=float(rcin.read(1))
        RCinput[1]=float(rcin.read(2))
        RCinput[2]=float(rcin.read(0))
        RCinput[3]=float(rcin.read(3))
        
        if i==1:
            c0=RCinput[0]
            c1=RCinput[1]
            c2=RCinput[2]
            c3=RCinput[3]
        else:
            c0=c0+(RCinput[0]-c0)/(i+1)
            c1=c1+(RCinput[1]-c1)/(i+1)
            c2=c2+(RCinput[2]-c2)/(i+1)
            c3=c3+(RCinput[3]-c3)/(i+1)

        i=i+1
        t2=time.time()
        x=t2-t1
            
    c0=int(c0)
    c1=int(c1)
    c2=int(c2)
    c3=int(c3)
    print('Average center points found to be (uS)')
    print('Ch0   Ch1   Ch2   Ch3')
    print('%d  %d  %d  %d' % (c0,c1,c2,c3))
    print(' ')
    print(' ')
    print('Step 2: Determine maximum and minimum PWM values from the RX.')
    print('        Move all sticks to their extremes within 10 seconds')
    print(' ')
    dummy=input('Press RETURN to start ')
    print(' ')
    print('Determining max and min PWM values...')
    print(' ')
    MAX_PWM=[1500,1500,1500,1500]
    MIN_PWM=[1500,1500,1500,1500]
    t1=time.time()
    x=0.0
    while x<10.0:
        # Get inputs from the receiver and save the max and min values
        # Definition is based of Assan X8R6
        RCinput[0]=float(rcin.read(1))
        RCinput[1]=float(rcin.read(2))
        RCinput[2]=float(rcin.read(0))
        RCinput[3]=float(rcin.read(3))

        if RCinput[0]>MAX_PWM[0]:
            MAX_PWM[0]=RCinput[0]
        if RCinput[0]<MIN_PWM[0]:
            MIN_PWM[0]=RCinput[0]
        
        if RCinput[1]>MAX_PWM[1]:
            MAX_PWM[1]=RCinput[1]
        if RCinput[1]<MIN_PWM[1]:
            MIN_PWM[1]=RCinput[1]
        
        if RCinput[2]>MAX_PWM[2]:
            MAX_PWM[2]=RCinput[2]
        if RCinput[2]<MIN_PWM[2]:
            MIN_PWM[2]=RCinput[2]
        
        if RCinput[3]>MAX_PWM[3]:
            MAX_PWM[3]=RCinput[3]
        if RCinput[3]<MIN_PWM[3]:
            MIN_PWM[3]=RCinput[3]

        t2=time.time()
        x=t2-t1

    print('Channel max and mins values collected (uS)')
    print('     Ch0   Ch1   Ch2   Ch3')
    print('MIN: %d  %d  %d  %d' % (MIN_PWM[0],MIN_PWM[1],MIN_PWM[2],MIN_PWM[3]))
    print('MAX: %d  %d  %d  %d' % (MAX_PWM[0],MAX_PWM[1],MAX_PWM[2],MAX_PWM[3]))
    print(' ')
    print('Determining slopes and intercepts based on:')
    print('    Pitch limit (deg): %d' % config.max_p)
    print('    Roll limit (deg): %d' % config.max_r)
    print('    Yaw_rate limit (deg/s): %d' % config.max_dy)
    print('    Dead band (uS): %d' % config.dead_band)
    print(' ')

    # PITCH
    # Determine slopes and intercepts
    mp_H=-config.max_p/(-MAX_PWM[1]+c1+config.dead_band)
    bp_H=(c1+config.dead_band)*config.max_p/(-MAX_PWM[1]+c1+config.dead_band)
    mp_L=config.max_p/(-MIN_PWM[1]+c1-config.dead_band)
    bp_L=-(c1-config.dead_band)*config.max_p/(-MIN_PWM[1]+c1-config.dead_band)

    # Find out which one is smaller and then compile the appropriate slope and intercepts
    if mp_H<=mp_L:
        bp_L2=-mp_H*(c1-config.dead_band)
        PTA_P=[mp_H,bp_L2,bp_H]
    elif mp_H>mp_L:
        bp_H2=-mp_L*(c1+config.dead_band)
        PTA_P=[mp_L,bp_L,bp_H2]

    # ROLL
    # Determine slopes and intercepts
    mr_H=-config.max_r/(-MAX_PWM[0]+c0+config.dead_band)
    br_H=(c0+config.dead_band)*config.max_r/(-MAX_PWM[0]+c0+config.dead_band)
    mr_L=config.max_r/(-MIN_PWM[0]+c0-config.dead_band)
    br_L=-(c0-config.dead_band)*config.max_r/(-MIN_PWM[0]+c0-config.dead_band)

    # Find out which one is smaller and then compile the appropriate slope and intercepts
    if mr_H<=mr_L:
        br_L2=-mr_H*(c0-config.dead_band)
        PTA_R=[mr_H,br_L2,br_H]
    elif mr_H>mr_L:
        br_H2=-mr_L*(c0+config.dead_band)
        PTA_R=[mr_L,br_L,br_H2]

    # YAW
    # Determine slopes and intercepts
    my_H=-config.max_dy/(-MAX_PWM[3]+c3+config.dead_band)
    by_H=(c3+config.dead_band)*config.max_dy/(-MAX_PWM[3]+c3+config.dead_band)
    my_L=config.max_dy/(-MIN_PWM[3]+c3-config.dead_band)
    by_L=-(c3-config.dead_band)*config.max_dy/(-MIN_PWM[3]+c3-config.dead_band)

    # Find out which one is smaller and then compile the appropriate slope and intercepts
    if my_H<=my_L:
        by_L2=-my_H*(c3-config.dead_band)
        PTA_Y=[my_H,by_L2,by_H]
    elif my_H>my_L:
        by_H2=-my_L*(c3+config.dead_band)
        PTA_Y=[my_L,by_L,by_H2]

    print('Slopes and intercepts for roll, pitch, and yaw_rate')
    print('PTA_R = %.3f %.3f %.3f' % (PTA_R[0],PTA_R[1],PTA_R[2]))
    print('PTA_P = %.3f %.3f %.3f' % (PTA_P[0],PTA_P[1],PTA_P[2]))
    print('PTA_Y = %.3f %.3f %.3f' % (PTA_Y[0],PTA_Y[1],PTA_Y[2]))
    print(' ')

    calib=open('QR_calib.txt', 'w')
    calib.write('%d %d %d %d\n' % (config.max_r,config.max_p,config.max_dy,config.dead_band))
    calib.write('%d %d %d %d\n' % (c0,c1,c2,c3))
    calib.write('%d %d %d %d\n' % (MIN_PWM[0],MIN_PWM[1],MIN_PWM[2],MIN_PWM[3]))
    calib.write('%d %d %d %d\n' % (MAX_PWM[0],MAX_PWM[1],MAX_PWM[2],MAX_PWM[3]))
    calib.write('%.4f %.4f %.4f\n' % (PTA_R[0],PTA_R[1],PTA_R[2]))
    calib.write('%.4f %.4f %.4f\n' % (PTA_P[0],PTA_P[1],PTA_P[2]))
    calib.write('%.4f %.4f %.4f' % (PTA_Y[0],PTA_Y[1],PTA_Y[2]))
    calib.close()

    print('Control calibration complete and calibration file written.')
    print(' ')
    print('Done.')

# ---------- Exit Sequence ---------- #
if (mode==1 or mode==4):
    if mode==4:
        flt_log.close()

    led.setColor('Black')
    AHRS_proc.join()
    ALT_proc.join()

    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Red')
    time.sleep(0.1)
    led.setColor('Blue')
    time.sleep(0.1)
    led.setColor('Black')
    print('Done.')
