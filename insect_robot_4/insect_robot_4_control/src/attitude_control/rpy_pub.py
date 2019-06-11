#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np
import rpy_algorithm as rpy

cycle_gait_data = Float32MultiArray()

if __name__ == '__main__':
    try:
        rospy.init_node('rpy_pub_node', anonymous=True)
        l_f_hip_joint_pos_pub = rospy.Publisher('/insect/l_f_hip_joint_position_controller/command', Float64, queue_size=10)
        r_f_hip_joint_pos_pub = rospy.Publisher('/insect/r_f_hip_joint_position_controller/command', Float64, queue_size=10)
        l_b_hip_joint_pos_pub = rospy.Publisher('/insect/l_b_hip_joint_position_controller/command', Float64, queue_size=10)
        r_b_hip_joint_pos_pub = rospy.Publisher('/insect/r_b_hip_joint_position_controller/command', Float64, queue_size=10)
        l_f_limb_joint_pos_pub = rospy.Publisher('/insect/l_f_limb_joint_position_controller/command', Float64, queue_size=10)
        r_f_limb_joint_pos_pub = rospy.Publisher('/insect/r_f_limb_joint_position_controller/command', Float64, queue_size=10)
        l_b_limb_joint_pos_pub = rospy.Publisher('/insect/l_b_limb_joint_position_controller/command', Float64, queue_size=10)
        r_b_limb_joint_pos_pub = rospy.Publisher('/insect/r_b_limb_joint_position_controller/command', Float64, queue_size=10)

        while not rospy.is_shutdown():
            j = 0
            default_angle = [0, 0, 0]
            RPY_angle_data_old = rospy.get_param('/insect/RPY_angle_old', default_angle)
            RPY_angle_data_new_traj = rospy.get_param('/insect/RPY_angle_new', default_angle)

            print(RPY_angle_data_old)
            print(RPY_angle_data_new_traj)
            if_traj_plan = False
            error = np.linalg.norm(np.array(RPY_angle_data_new_traj))
            print(error)
            if (error>0.05):
                if if_traj_plan:
                    rate, cycle_gait_np_data, RPY_B_traj = rpy.rpy_gait(RPY_angle_data_old, RPY_angle_data_new_traj)
                    data_length = cycle_gait_np_data.shape[0]
                    pause = rospy.Rate(data_length * rate)
                    cycle_gait_data.data = cycle_gait_np_data
                    while (j<1):
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
                else:
                    rate, cycle_gait_np_data = rpy.rpy_adj(RPY_angle_data_new_traj)
                    data_length = cycle_gait_np_data.shape[0]
                    pause = rospy.Rate(data_length * rate)
                    cycle_gait_data.data = cycle_gait_np_data
                    print(cycle_gait_np_data)

                    l_f_hip_joint_pos_pub.publish(cycle_gait_data.data[0, 0])
                    r_f_hip_joint_pos_pub.publish(cycle_gait_data.data[0, 1])
                    l_b_hip_joint_pos_pub.publish(cycle_gait_data.data[0, 2])
                    r_b_hip_joint_pos_pub.publish(cycle_gait_data.data[0, 3])
                    l_f_limb_joint_pos_pub.publish(cycle_gait_data.data[0, 4])
                    r_f_limb_joint_pos_pub.publish(cycle_gait_data.data[0, 5])
                    l_b_limb_joint_pos_pub.publish(cycle_gait_data.data[0, 6])
                    r_b_limb_joint_pos_pub.publish(cycle_gait_data.data[0, 7])
                    
                    pause.sleep()


            else:
                pause = rospy.Rate(1)
                pause.sleep()

            rospy.set_param('/insect/RPY_angle_old', RPY_angle_data_new_traj)
            rospy.set_param('/insect/in_wait', True)
    except rospy.ROSInterruptException:
        pass
