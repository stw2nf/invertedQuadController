#!/usr/bin/env python
import rospy
import numpy as np
from navio2ros.msg import RC # for reading in RC values from TX
from navio2ros.msg import PWM # for outputting values to the servo rail
from navio2ros.msg import ADC
from navio2ros.msg import PID
from navio2ros.msg import IMU
from navio2ros.msg import AHRS
from navio2ros.msg import Angular
from navio2ros.msg import Barometer
import complete_functions as func

#Setup subscibers
sub_madgwick = rospy.Subscriber('/madgwickpub', AHRS, func.callback_madgwick)
sub_imu = rospy.Subscriber('/imumpupub', IMU, func.callback_imu)
sub_adc = rospy.Subscriber('/adcpub', ADC, func.callback_adc)
sub_barometer = rospy.Subscriber('/baropub', Barometer, func.callback_barometer)
sub_rc = rospy.Subscriber('/rcpub', RC, func.callback_rc)

#Setup publishers
pub_pid = rospy.Publisher('/pid', PID, queue_size=26)
pub_pwm = rospy.Publisher('/motorcommand', PWM, queue_size=4)

# register the node
rospy.init_node('normal_cascade_invert_enabled')

#PWM and PID class for outputs
pwmout = PWM()
pid = PID()

#Setting initial error values
erollrate = np.zeros(2)
derollrate = 0
ierollrate = np.zeros(2)
traprollrate = 0
erollangle = np.zeros(2)
derollangle = 0
ierollangle = np.zeros(2)
traprollangle = 0

epitchrate = np.zeros(2)
depitchrate = 0
iepitchrate = np.zeros(2)
trappitchrate = 0
epitchangle = np.zeros(2)
depitchangle = 0
iepitchangle = np.zeros(2)
trappitchangle = 0

eyawrate = np.zeros(2)
deyawrate = 0
ieyawrate = np.zeros(2)
trapyawrate = 0
 
#Initalizing PID for roll and rate
Prollrate = 0
Drollrate = 0
Irollrate = 0
Prollangle = 0
Drollangle = 0
Irollangle = 0

Ppitchrate = 0
Dpitchrate = 0
Ipitchrate = 0
Ppitchangle = 0
Dpitchangle = 0
Ipitchangle = 0

Pyawrate = 0
Dyawrate = 0
Iyawrate = 0

#Setting kp, kd, and ki for Inverted Flight
kprate_invert = 1.6
kdrate_invert = .075
kirate_invert = 0

kpangle_invert = 2.25
kdangle_invert = 0.07
kiangle_invert = 0

#Set commands to zero
rollcmd = 0
pitchcmd = 0
yawcmd = 0

#Set initial arrays to zero for filtering rate data (moving average)
uf_rollrate = np.zeros(4)
uf_pitchrate = np.zeros(4)
uf_yawrate = np.zeros(4)
uf_rollangle = np.zeros(4)
uf_pitchangle = np.zeros(4)

#Set limits so commands dont go crazy due to noise
rollpitchrate_ceiling = 175
rollpitchangle_ceiling = 50
yawrate_ceiling = 100

#Setting rate
rate = rospy.Rate(100)
dt = .01
tflip = 0
tflip_total = 1.0

#Find Elevation of ground
dt = .01
stop = 2
steps = int(stop/dt)
elevation_ground = np.zeros(steps)

for i in range(steps):
    elevation_ground[i] = func.msl_elevation
    rate.sleep()

elevation_bias = np.mean(elevation_ground)

#3 Position mode switch values can be 1088, 1514 and 1940
#1088 = Normal Flight Mode
#1514 = Inverted Flight Mode
#1940 = Flip back to Normal Flight Mode (Rescue Command)

if __name__ == '__main__':

	# try/except block here is a fancy way to allow code to cleanly exit on a keyboard break (ctrl+c)
	try:
		while not rospy.is_shutdown():

			#Set everything to zero if the kill switch is flipped
			if func.killswitch > 1500:

				#Resets integral error with kill switch
				ierollrate = np.zeros(2)
                		iepitchrate = np.zeros(2)
                		ieyawrate = np.zeros(2)
				ierollangle = np.zeros(2)
                		iepitchangle = np.zeros(2)
                		ieyawangle = np.zeros(2)
				tflip = 0

    		    		for i in range(len(pwmout.channel)):
				    pwmout.channel[i] = 1.0 # rc values are integers (1000-2000), we want 1.0-2.0
				
				# publish the topic to motor command
				pub_pwm.publish(pwmout)

				rate.sleep()

			#Go live if kill switch is not flipped
			elif func.killswitch < 1500:
				
				#Setting kp, kd, ki for Normal Flight
				kprollrate = 1.299
				kdrollrate = .0085
				kirollrate = 0.25
				kprollangle = 3.0
				kdrollangle = 0.006
				kirollangle = 0

				kppitchrate = 1.299
				kdpitchrate = 0.0085
				kipitchrate = 0.25
				kppitchangle = 2.25
				kdpitchangle = 0.00175
				kipitchangle = 0

				kpyawrate = 1.75
				kdyawrate = 0
				kiyawrate = 0

				#Read in inverted desired and actual angles for inverted flight mode
				roll_angle_desired_invert = func.rc_roll_invert
				roll_angle_actual_invert = func.roll_angle_invert

				#Read in desired and actual angles for normal flight mode
				roll_angle_desired_normal = func.rc_roll
				roll_angle_actual_normal = func.roll_angle

				#Decide which angles to use (based on which flight mode I am in)
				roll_angle_desired = roll_angle_desired_normal
				roll_angle_actual = roll_angle_actual_normal

				if func.flightmode > 20000:

					#Start count for flip time
					tflip = tflip + dt

					#Read in inverted RC and inverted angle reading
					roll_angle_desired = roll_angle_desired_invert
					roll_angle_actual = roll_angle_actual_invert

					#Reset gains to invert gains if we are in inverted flight mode
					if tflip > tflip_total:
						
						kprollrate = kprate_invert
						kdrollrate = kdrate_invert
						kirollrate = kirate_invert

						kprollangle = kpangle_invert
						kdrollangle = kdangle_invert
						kirollangle = kiangle_invert

						kppitchrate = kprate_invert
						kdpitchrate = kdrate_invert
						kipitchrate = kirate_invert

						kppitchangle = kpangle_invert
						kdpitchangle = kdangle_invert
						kipitchangle = kiangle_invert

				#Calculate Errors for roll and pitch				
				erollangle[0] = roll_angle_desired - roll_angle_actual
				derollangle = (erollangle[0]-erollangle[1])/dt
				traprollangle = ((erollangle[0]+erollangle[1])/2.0)*dt
				ierollangle[0] = ierollangle[1]+traprollangle
                
				pitch_angle_desired = func.rc_pitch
				pitch_angle_actual = func.pitch_angle
                		epitchangle[0] = pitch_angle_desired - pitch_angle_actual
				depitchangle = (epitchangle[0]-epitchangle[1])/dt
				trappitchangle = ((epitchangle[0]+epitchangle[1])/2.0)*dt
				iepitchangle[0] = iepitchangle[1]+trappitchangle

				#Roll Angle Ceiling (Axis for flip)
				if abs(erollangle[0]) > rollpitchangle_ceiling:
					if erollangle[0] > 0:
						erollangle[0] = rollpitchangle_ceiling
					if erollangle[0] < 0:
						erollangle[0] = -rollpitchangle_ceiling

				#Calculate PID for Roll and Pitch
				Prollangle = kprollangle*erollangle[0]
				Drollangle = kdrollangle*derollangle
				Irollangle = kirollangle*ierollangle[0]
               			roll_rate_cmd = Prollangle + Drollangle + Irollangle

                		Ppitchangle = kppitchangle*epitchangle[0]
				Dpitchangle = kdpitchangle*depitchangle
				Ipitchangle = kipitchangle*iepitchangle[0]
               			pitch_rate_cmd = Ppitchangle + Dpitchangle + Ipitchangle

				#Cascade into rate controller
				roll_rate_desired = roll_rate_cmd
				uf_rollrate[0] = func.rate_roll
				roll_rate_actual = func.movingaverage(uf_rollrate[0], uf_rollrate[1], uf_rollrate[2], uf_rollrate[3])

                		pitch_rate_desired = pitch_rate_cmd
				uf_pitchrate[0] = func.rate_pitch
				pitch_rate_actual = func.movingaverage(uf_pitchrate[0], uf_pitchrate[1], uf_pitchrate[2], uf_pitchrate[3])
                
                		yaw_rate_desired = func.rc_yawrate
				uf_yawrate[0] = func.rate_yaw
				yaw_rate_actual = func.movingaverage(uf_yawrate[0], uf_yawrate[1], uf_yawrate[2], uf_yawrate[3])

				#Setting ceilings so the motors dont max out
				if abs(roll_rate_actual) > rollpitchrate_ceiling:
					if roll_rate_actual > 0:
						roll_rate_actual = rollpitchrate_ceiling
					if roll_rate_actual < 0:
						roll_rate_actual = -rollpitchrate_ceiling

				if abs(pitch_rate_actual) > rollpitchrate_ceiling:
					if pitch_rate_actual > 0:
						pitch_rate_actual = rollpitchrate_ceiling
					if pitch_rate_actual < 0:
						pitch_rate_actual = -rollpitchrate_ceiling

				if abs(yaw_rate_actual) > yawrate_ceiling:
					if yaw_rate_actual > 0:
						yaw_rate_actual = yawrate_ceiling
					if yaw_rate_actual < 0:
						yaw_rate_actual = -yawrate_ceiling

				#Calculating the dt, Error, Derivative Error, and integral error for Roll, pitch and yaw rate
				erollrate[0] = roll_rate_desired - roll_rate_actual
				derollrate = (erollrate[0]-erollrate[1])/dt
				traprollrate = ((erollrate[0]+erollrate[1])/2.0)*dt
				ierollrate[0] = ierollrate[1]+traprollrate
                
                		epitchrate[0] = pitch_rate_desired - pitch_rate_actual
				depitchrate = (epitchrate[0]-epitchrate[1])/dt
				trappitchrate = ((epitchrate[0]+epitchrate[1])/2.0)*dt
				iepitchrate[0] = iepitchrate[1]+trappitchrate
                
                		eyawrate[0] = yaw_rate_desired - yaw_rate_actual
				deyawrate = (eyawrate[0]-eyawrate[1])/dt
				trapyawrate = ((eyawrate[0]+eyawrate[1])/2.0)*dt
				ieyawrate[0] = ieyawrate[1]+trapyawrate
				
				#Calculate PID for Roll
				Prollrate = kprollrate*erollrate[0]
				Drollrate = kdrollrate*derollrate
				Irollrate = kirollrate*ierollrate[0]
               
                		Ppitchrate = kppitchrate*epitchrate[0]
				Dpitchrate = kdpitchrate*depitchrate
				Ipitchrate = kipitchrate*iepitchrate[0]
				
                		Pyawrate = kpyawrate*eyawrate[0]
				Dyawrate = kdyawrate*deyawrate
				Iyawrate = kiyawrate*ieyawrate[0]
                
                		#Calculate Motor Commands for Roll and Pitch
				rollcmd = Prollrate + Drollrate + Irollrate
				pitchcmd = Ppitchrate + Dpitchrate + Ipitchrate				
				yawcmd = Pyawrate + Dyawrate + Iyawrate
				throttlecmd = func.rc_throttle

				#Set Baseline throttle for inverted flight mode
				if func.flightmode > 20000:
					throttlecmd = 1460

				#Setting Commands for motor for normal flight, inverted flight and flipping
				M1_cmd = throttlecmd - pitchcmd - yawcmd
				M2_cmd = throttlecmd - rollcmd + yawcmd
                		M3_cmd = throttlecmd + pitchcmd - yawcmd
                		M4_cmd = throttlecmd + rollcmd + yawcmd
				
				#Pitch and yaw commands flip for inverted case
				if func.flightmode > 2000:
					M1_cmd = throttlecmd + pitchcmd + yawcmd
					M2_cmd = throttlecmd - rollcmd - yawcmd
					M3_cmd = throttlecmd - pitchcmd + yawcmd
                			M4_cmd = throttlecmd + rollcmd - yawcmd

				#Publish normal flight commands for normal flight mode
				pwmout.channel[0] = M1_cmd/1000
				pwmout.channel[1] = M2_cmd/1000				
				pwmout.channel[2] = M3_cmd/1000
				pwmout.channel[3] = M4_cmd/1000

				#Update values before iterating again			
				erollrate[1] = erollrate[0]
				ierollrate[1] = ierollrate[0]

				erollangle[1] = erollangle[0]
				ierollangle[1] = ierollangle[0]

				uf_rollrate[3] = uf_rollrate[2]
				uf_rollrate[2] = uf_rollrate[1]
				uf_rollrate[1] = uf_rollrate[0]
                
                		epitchrate[1] = epitchrate[0]
				iepitchrate[1] = iepitchrate[0]

                		epitchangle[1] = epitchangle[0]
				iepitchangle[1] = iepitchangle[0]

				uf_pitchrate[3] = uf_pitchrate[2]
				uf_pitchrate[2] = uf_pitchrate[1]
				uf_pitchrate[1] = uf_pitchrate[0]
                
                		eyawrate[1] = eyawrate[0]
				ieyawrate[1] = ieyawrate[0]
				
				#Calculate Power Consumption
				powerconsumption = func.adc.pwrportvoltage*func.adc.pwrportcrnt

				#Calculate elevation AGL (feet)
				elevation_agl = func.msl_elevation - elevation_bias
				#print(elevation_agl)

				#Set values to output on the PID message
				pid.desired_roll_angle = roll_angle_desired
				pid.actual_roll_angle = roll_angle_actual 
				pid.rollcmd = rollcmd
				pid.prollangle = Prollangle
				pid.drollangle = Drollangle
				pid.irollangle = Irollangle
                
				pid.desired_pitch_angle = pitch_angle_desired
				pid.actual_pitch_angle = pitch_angle_actual
				pid.pitchcmd = pitchcmd
				pid.ppitchangle = Ppitchangle
				pid.dpitchangle = Dpitchangle
				pid.ipitchangle = Ipitchangle
                
				pid.desired_yaw_rate = func.rc_yawrate
				pid.actual_yaw_rate = func.rate_yaw
				pid.yawcmd = yawcmd
 				pid.pyawrate = Pyawrate
				pid.dyawrate = Dyawrate
				pid.iyawrate = Iyawrate
				
				pid.powerconsumption = powerconsumption

				pid.elevation_agl = elevation_agl

				pub_pid.publish(pid)

				# write the pwmout values, using outval as the commmand for all channels
				for i in range(len(pwmout.channel)):
				    pwmout.channel[i] = pwmout.channel[i] # rc values are integers (1000-2000), we want 1.0-2.0

				pub_pwm.publish(pwmout)

				rate.sleep()


	# as stated before, try/except is used to nicely quit the program using ctrl+c
	except rospy.ROSInterruptException:

		# before shutting down, turn all outputs back to 1 for safety
		for i in range(len(pwmout.channel)):
			pwmout.channel[i] = 1.0

		# publish the topic before closing
		pub_pwm.publish(pwmout)

		pass
