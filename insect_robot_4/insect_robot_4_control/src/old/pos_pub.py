#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np
import kinematics_algorithm as ka

def pos_pub_cb(action_command_msg):
    j = 0
    cycle_gait_data = Float32MultiArray()
    if (action_command_msg.data == 'w'):
        rate, cycle_gait_np_data = ka.forward_gait()
    elif (action_command_msg.data == 's'):
        rate, cycle_gait_np_data = ka.backward_gait()
    elif (action_command_msg.data == 'a'):
        rate, cycle_gait_np_data = ka.turnleft_gait()
    elif (action_command_msg.data == 'd'):
        rate, cycle_gait_np_data = ka.turnright_gait()
    elif (action_command_msg.data == 'j'):
        rate, cycle_gait_np_data = ka.jump_gait()
        j = 1
    elif (action_command_msg.data == 'k'):
        rate, cycle_gait_np_data = ka.keep_gait()
        j = 1
    rospy.loginfo(cycle_gait_np_data)
    data_length = cycle_gait_np_data.shape[0]
    pause = rospy.Rate(data_length * rate)
    cycle_gait_data.data = cycle_gait_np_data
    while (j<2):
        for i in range(data_length):
            l_f_hip_joint_pos_pub.publish(cycle_gait_data.data[i, 0])
            r_f_hip_joint_pos_pub.publish(cycle_gait_data.data[i, 1])
            l_b_hip_joint_pos_pub.publish(cycle_gait_data.data[i, 2])
            r_b_hip_joint_pos_pub.publish(cycle_gait_data.data[i, 3])
            l_f_limb_joint_pos_pub.publish(cycle_gait_data.data[i, 4])
            r_f_limb_joint_pos_pub.publish(cycle_gait_data.data[i, 5])
            l_b_limb_joint_pos_pub.publish(cycle_gait_data.data[i, 6])
            r_b_limb_joint_pos_pub.publish(cycle_gait_data.data[i, 7])
            pause.sleep()
        j = j + 1

def stay_in_place():
    zeros = Float64()
    zeros.data = 0.2
    l_f_hip_joint_pos_pub.publish(0)
    r_f_hip_joint_pos_pub.publish(0)
    l_b_hip_joint_pos_pub.publish(0)
    r_b_hip_joint_pos_pub.publish(0)
    l_f_limb_joint_pos_pub.publish(zeros.data * 2)
    r_f_limb_joint_pos_pub.publish(zeros.data * 2)
    l_b_limb_joint_pos_pub.publish(zeros.data)
    r_b_limb_joint_pos_pub.publish(zeros.data)


if __name__ == '__main__':
    try:
        rospy.init_node('pos_pub_node', anonymous=True)
        l_f_hip_joint_pos_pub = rospy.Publisher('/insect/l_f_hip_joint_position_controller/command', Float64, queue_size=10)
        r_f_hip_joint_pos_pub = rospy.Publisher('/insect/r_f_hip_joint_position_controller/command', Float64, queue_size=10)
        l_b_hip_joint_pos_pub = rospy.Publisher('/insect/l_b_hip_joint_position_controller/command', Float64, queue_size=10)
        r_b_hip_joint_pos_pub = rospy.Publisher('/insect/r_b_hip_joint_position_controller/command', Float64, queue_size=10)
        l_f_limb_joint_pos_pub = rospy.Publisher('/insect/l_f_limb_joint_position_controller/command', Float64, queue_size=10)
        r_f_limb_joint_pos_pub = rospy.Publisher('/insect/r_f_limb_joint_position_controller/command', Float64, queue_size=10)
        l_b_limb_joint_pos_pub = rospy.Publisher('/insect/l_b_limb_joint_position_controller/command', Float64, queue_size=10)
        r_b_limb_joint_pos_pub = rospy.Publisher('/insect/r_b_limb_joint_position_controller/command', Float64, queue_size=10)
        rospy.Subscriber("action_command", String, pos_pub_cb)
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
