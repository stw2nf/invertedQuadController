# -*- coding: utf-8 -*-

import rospy
import numpy as np

class adc:
    def __init__ (self, board, servorail, pwrportvoltage, pwrportcrnt, adc2, adc3):
        self.board = board
        self.servorail = servorail
        self.pwrportvoltage = pwrportvoltage
        self.pwrportcrnt = pwrportcrnt
        self.adc2 = adc2
        self.adc3 = adc3

def callback_imu(data):
    rollbias = -.002
    pitchbias = -.05
    yawbias = -.03
    global rate_roll
    rate_roll = (data.gyroscope.x*(180/np.pi))-rollbias
    global rate_pitch
    rate_pitch = (-data.gyroscope.y*(180/np.pi))-pitchbias
    global rate_yaw
    rate_yaw = (data.gyroscope.z*(180/np.pi))-yawbias

def callback_barometer(data):
    #Elevation in feet (MSL)
    global msl_elevation
    msl_elevation = data.elevation

def callback_gps(data):
    global gps_longitude
    global gps_latitude
    gps_longitude = data.longitude
    gps_latitude = data.latitude


def callback_madgwick(data):
    rollanglebias = 1.7
    pitchanglebias = .4
    global roll_angle
    global roll_angle_invert
    global pitch_angle
    roll_angle = data.angular.roll - rollanglebias
    pitch_angle = -data.angular.pitch - pitchanglebias
    roll_angle_invert = data.angular.roll
    if data.angular.roll < 0:
         roll_angle_invert = 360 + data.angular.roll

def callback_adc(data):
    
    global bits2volt
    global bits2amp
    global ampoffset
    global voltoffset
    global encodersig2deg
    global adc

    encodersig2deg = .072405
    bits2volt =.0109
    bits2amp = .1456
    ampoffset = 1.0499
    voltoffset = 0.0898
    adc.board = data.channel[0]
    adc.servorail = data.channel[1]
    adc.pwrportvoltage = (data.channel[2]*bits2volt)+voltoffset
    adc.pwrportcrnt = (data.channel[3]*bits2amp)+ampoffset
    adc.adc2 = -data.channel[4]*encodersig2deg
    adc.adc3 = data.channel[5]
    
# use this callback to get exactly one channel from the rc message and copy
# its value into a variable called outval
def callback_rc(data):
    roll_channel = 0
    pitch_channel = 1
    throttle_channel = 2
    yaw_channel = 3
    killswitch_channel = 4
    flightmode_channel = 5
    global maxdeg
    minsig = 1088.0
    maxsig = 1940.0
    maxdeg = 30
    mindeg = -30
    mindeg_invert = 150
    maxdeg_invert = 210
    maxrate = 90
    maxrateyaw = 90
    minrate = -90 
    minrateyaw = -90
    global rc_roll
    rc_roll=(((maxdeg-mindeg)*((maxsig-data.channel[roll_channel])/(maxsig-minsig)))+mindeg)
    global rc_roll_invert
    rc_roll_invert = (((maxdeg_invert-mindeg_invert)*((maxsig-data.channel[roll_channel])/(maxsig-minsig)))+mindeg_invert)
    global rc_rollrate
    rc_rollrate=(((maxrate-minrate)*((maxsig-data.channel[roll_channel])/(maxsig-minsig)))+minrate)
    global rc_pitch
    rc_pitch=-(((maxdeg-mindeg)*((maxsig-data.channel[pitch_channel])/(maxsig-minsig)))+mindeg)
    global rc_pitchrate
    rc_pitchrate=-(((maxrate-minrate)*((maxsig-data.channel[pitch_channel])/(maxsig-minsig)))+minrate)
    global rc_yawrate
    rc_yawrate=((minrateyaw-maxrateyaw)/(maxsig-minsig))*(data.channel[yaw_channel]-minsig)+maxrateyaw
    global rc_throttle
    rc_throttle=(data.channel[throttle_channel])
    global killswitch
    killswitch = data.channel[killswitch_channel]
    global flightmode
    flightmode = data.channel[flightmode_channel]

def callback_pwm(data):
    M1_channel = 0
    M2_channel = 1
    M3_channel = 2
    M4_channel = 3
    global M1_command
    M1_command = data.channel[M1_channel]
    global M2_command
    M2_command = data.channel[M2_channel]
    global M3_command
    M3_command = data.channel[M3_channel]
    global M4_command
    M4_command = data.channel[M4_channel]
   
def callback_pid(data):
    global desired_roll_angle
    desired_roll_angle = data.desired_roll_angle
    global actual_roll_angle
    actual_roll_angle = data.actual_roll_angle
    global rollcmd
    rollcmd = data.rollcmd
    global prollangle
    prollangle = data.prollangle
    global drollangle
    drollangle = data.drollangle
    global irollangle
    irollangle = data.irollangle
    global prollrate
    prollrate = data.prollrate
    global drollrate
    drollrate = data.drollrate
    global irollrate
    irollrate = data.irollrate
    global desired_pitch_angle
    desired_pitch_angle = data.desired_pitch_angle
    global actual_pitch_angle
    actual_pitch_angle = data.actual_pitch_angle
    global pitchcmd
    pitchcmd = data.pitchcmd
    global ppitchangle
    ppitchangle = data.ppitchangle
    global dpitchangle
    dpitchangle = data.dpitchangle
    global ipitchangle
    ipitchangle = data.ipitchangle
    global ppitchrate
    ppitchrate = data.ppitchrate
    global dpitchrate
    dpitchrate = data.dpitchrate
    global ipitchrate
    ipitchrate = data.ipitchrate
    global desired_yaw_rate
    desired_yaw_rate = data.desired_yaw_rate
    global actual_yaw_rate
    actual_yaw_rate = data.actual_yaw_rate
    global yawcmd
    yawcmd = data.yawcmd
    global pyawrate
    pyawrate = data.pyawrate
    global dyawrate
    dyawrate = data.dyawrate
    global iyawrate
    iyawrate = data.iyawrate
    global powerconsumption
    powerconsumption = data.powerconsumption
    global elevation_agl
    elevation_agl = data.elevation_agl

def lowpassfilter2(uf_current, uf_old, uf_oldold, f_old, f_oldold):
    
    #Defining coefficients
    fc = 8.0
    fs = 100.0
    y = np.tan((np.pi*fc)/fs)
    D = (y**2)+(np.sqrt(2)*y)+1
    b0 = (y**2)/D
    b1 = (2*(y**2))/D
    b2 = (y**2)/D
    a1 = (2*((y**2)-1))/D
    a2 = (y**2)-(np.sqrt(2)*y)+1
    
    #calculate the unfiltered value
    f_current = (b0*uf_current)+(b1*uf_old)+(b2*uf_oldold)-(a1*f_old)-(a2*f_oldold)
    
    return f_current

def lowpassfilter3(uf_current, uf_old, uf_oldold, uf_oldoldold, f_old, f_oldold, f_oldoldold):
    
    #Defining coefficients
    fc = 7.0
    fs = 150.0
    y = np.tan((np.pi*fc)/fs)
    D = (y**3)+(2*(y**2))+(2*y)+1
    b0 = (y**3)/D
    b1 = (3*(y**3))/D
    b2 = (3*(y**3))/D
    b3 = (y**3)/D
    a1 = ((3*(y**3))+(2*(y**2))-(2*y)-3)/D
    a2 = ((3*(y**3))-(2*(y**2))-(2*y)+3)/D
    a3 = ((y**3)-(2*(y**2))+(2*y)-1)/D

    #calculate the unfiltered value
    f_current = (b0*uf_current)+(b1*uf_old)+(b2*uf_oldold)+(b3*uf_oldoldold)-(a1*f_old)-(a2*f_oldold)-(a3*f_oldoldold)
    
    return f_current

def movingaverage(uf_current, uf_old, uf_oldold, uf_oldoldold):
    
    smoothed_current = (uf_current + uf_old + uf_oldold + uf_oldoldold)/4
    return smoothed_current
