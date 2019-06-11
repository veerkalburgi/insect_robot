#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np
import kinematics_algorithm as ka



def action_pub(gait_data, data_length):
    j = 0
    while (j<1):
        for i in range(data_length):
            l_f_hip_joint_pos_pub.publish(gait_data.data[i, 0])
            r_f_hip_joint_pos_pub.publish(gait_data.data[i, 1])
            l_b_hip_joint_pos_pub.publish(gait_data.data[i, 2])
            r_b_hip_joint_pos_pub.publish(gait_data.data[i, 3])
            l_f_limb_joint_pos_pub.publish(gait_data.data[i, 4])
            r_f_limb_joint_pos_pub.publish(gait_data.data[i, 5])
            l_b_limb_joint_pos_pub.publish(gait_data.data[i, 6])
            r_b_limb_joint_pos_pub.publish(gait_data.data[i, 7])

            pause.sleep()
        j = j + 1
    return

def command_analysis(action_command):

    if (action_command == 'w'):
        rate, gait_np_data = ka.forward_gait()
    elif (action_command == 's'):
        rate, gait_np_data = ka.backward_gait()
    elif (action_command == 'a'):
        rate, gait_np_data = ka.turnleft_gait()
    elif (action_command == 'd'):
        rate, gait_np_data = ka.turnright_gait()
    elif (action_command == 'j'):
        rate, gait_np_data = ka.jump_gait()
    elif (action_command == 'k'):
        rate, gait_np_data = ka.keep_gait()
    elif (action_command == 'c'):
        rate, gait_np_data = ka.clam_gait()
    elif (action_command == 'q'):
        rate, gait_np_data = ka.slantleft_gait()
    elif (action_command == 'e'):
        rate, gait_np_data = ka.slantright_gait()
    return rate, gait_np_data

if __name__ == '__main__':
    try:
        # Initialize the node and define the Publisher.
        rospy.init_node('pos_pub_node', anonymous=True)
        l_f_hip_joint_pos_pub = rospy.Publisher('/insect/l_f_hip_joint_position_controller/command', Float64, queue_size=10)
        r_f_hip_joint_pos_pub = rospy.Publisher('/insect/r_f_hip_joint_position_controller/command', Float64, queue_size=10)
        l_b_hip_joint_pos_pub = rospy.Publisher('/insect/l_b_hip_joint_position_controller/command', Float64, queue_size=10)
        r_b_hip_joint_pos_pub = rospy.Publisher('/insect/r_b_hip_joint_position_controller/command', Float64, queue_size=10)
        l_f_limb_joint_pos_pub = rospy.Publisher('/insect/l_f_limb_joint_position_controller/command', Float64, queue_size=10)
        r_f_limb_joint_pos_pub = rospy.Publisher('/insect/r_f_limb_joint_position_controller/command', Float64, queue_size=10)
        l_b_limb_joint_pos_pub = rospy.Publisher('/insect/l_b_limb_joint_position_controller/command', Float64, queue_size=10)
        r_b_limb_joint_pos_pub = rospy.Publisher('/insect/r_b_limb_joint_position_controller/command', Float64, queue_size=10)
        
        while not rospy.is_shutdown():
            # Read action command.
            action_command = rospy.get_param('/insect/action_state_param', 'k')

            # Analyze the action command and do gait planning. Note that the gait data returned here is a numpy array.
            rate, gait_np_data = command_analysis(action_command)

            # Calculate the pause time for each step of publish to make the total publish frequency equal to the "rate" in gait planning.
            # Note that a total publish contains 40 steps of publish. (Number of nodes in the gait planning)
            data_length = gait_np_data.shape[0]
            pause = rospy.Rate(data_length * rate)

            # Assign the gait data to gait_data and publish.
            gait_data = Float32MultiArray() # Define the gait data as std_msgs.msg data because it is to be published to the topic.
            gait_data.data = gait_np_data
            action_pub(gait_data, data_length)

    except rospy.ROSInterruptException:
        pass
