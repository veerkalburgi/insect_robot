#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs


#Define a Insect robot joint positions publisher for joint controllers.
def insect_joint_positions_publisher():

	#Initiate node for controlling joint1 and joint2 positions.
	rospy.init_node('joint_positions_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pub1 = rospy.Publisher('/insect/l_f_hip_joint_position_controller/command', Float64, queue_size=10)
	pub2 = rospy.Publisher('/insect/r_f_hip_joint_position_controller/command', Float64, queue_size=10)
	pub3 = rospy.Publisher('/insect/l_b_hip_joint_position_controller/command', Float64, queue_size=10)
	pub4 = rospy.Publisher('/insect/r_b_hip_joint_position_controller/command', Float64, queue_size=10)
	pub5 = rospy.Publisher('/insect/l_f_limb_joint_position_controller/command', Float64, queue_size=10)
	pub6 = rospy.Publisher('/insect/r_f_limb_joint_position_controller/command', Float64, queue_size=10)
	pub7 = rospy.Publisher('/insect/l_b_limb_joint_position_controller/command', Float64, queue_size=10)
	pub8 = rospy.Publisher('/insect/r_b_limb_joint_position_controller/command', Float64, queue_size=10)








	rate = rospy.Rate(100) #100 Hz

	#While loop to have joints follow a certain position, while rospy is not shutdown.
	i = 0
	while not rospy.is_shutdown():

		#Have each joint follow a sine movement of sin(i/100).
		sine_movement = sin(i/100)
		cose_movement = cos(i/100)


		#Publish the same sine movement to each joint.
		pub1.publish(sine_movement)
		pub3.publish(sine_movement)
		pub5.publish(sine_movement)
		pub7.publish(sine_movement)
		rate.sleep()
		pub2.publish(sine_movement)
		pub4.publish(sine_movement)
		pub6.publish(sine_movement)
		pub8.publish(sine_movement)

		i = i+1 #increment i

		rate.sleep() #sleep for rest of rospy.Rate(100)


#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: insect_joint_positions_publisher()
	except rospy.ROSInterruptException: pass
