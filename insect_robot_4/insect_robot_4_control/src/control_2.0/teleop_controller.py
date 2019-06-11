#!/usr/bin/env python
# -*- coding: utf-8 -*

import  os
import  sys
import  tty, termios
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float64


# Global variable
pub_action = rospy.Publisher('/insect/action_chatter', String, queue_size=10)
action_msgs = String()

def keyboardLoop():
    #initialization
    rospy.init_node('smartcar_teleop')
    rate = rospy.Rate(1)

    #Display prompt message
    print("Reading from keyboard")
    print("Use WASD keys to control the robot")
    print("Press Caps to move faster")
    print("Press q to quit")

    #Read button cycle
    while not rospy.is_shutdown():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
		#Does not produce an echo effect
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try :
            tty.setraw( fd )
            ch = sys.stdin.read( 1 )
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if (ch == 'p'):
          exit()
        else:
          action_msgs.data = ch
          pub_action.publish(action_msgs)
        rate.sleep()

if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
