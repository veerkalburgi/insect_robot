#!/usr/bin/env python

import rospy
import time
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class insectJointMover(object):

    def __init__(self):
        rospy.init_node('jointmover_demo', anonymous=True)
        rospy.loginfo("Insect JointMover Initialising...")

        self.pub_l_f_hip_joint_position = rospy.Publisher(
            '/insect/l_f_hip_joint_position_controller/command',
            Float64,
            queue_size=10)
        self.pub_r_f_hip_joint_position = rospy.Publisher(
            '/insect/r_f_hip_joint_position_controller/command',
            Float64,
            queue_size=10)
        self.pub_l_b_hip_joint_position = rospy.Publisher(
            '/insect/l_b_hip_joint_position_controller/command',
            Float64,
            queue_size=10)
        self.pub_r_b_hip_joint_position = rospy.Publisher(
            '/insect/r_b_hip_joint_position_controller/command',
            Float64,
            queue_size=10)
        self.pub_l_f_limb_joint_position = rospy.Publisher(
            '/insect/l_f_limb_joint_position_controller/command',
            Float64,
            queue_size=10)
        self.pub_r_f_limb_joint_position = rospy.Publisher(
            '/insect/r_f_limb_joint_position_controller/command',
            Float64,
            queue_size=10)
        self.pub_l_b_limb_joint_position = rospy.Publisher(
            '/insect/l_b_limb_joint_position_controller/command',
            Float64,
            queue_size=10)
        self.pub_r_b_limb_joint_position = rospy.Publisher(
            '/insect/r_b_limb_joint_position_controller/command',
            Float64,
            queue_size=10)


        joint_states_topic_name = "/insect/joint_states"
        rospy.Subscriber(joint_states_topic_name, JointState, self.insect_joints_callback)
        insect_joints_data = None
        rate = rospy.Rate(2)
        while insect_joints_data is None:
            try:
                insect_joints_data = rospy.wait_for_message(joint_states_topic_name, JointState, timeout=5)
            except:
                rospy.logwarn("Time out " + str(joint_states_topic_name))
                pass
            rate.sleep()

        self.insect_joint_dictionary = dict(zip(insect_joints_data.name, insect_joints_data.position))

    def move_insect_all_joints(self, l_f_hip_angle, r_f_hip_angle, l_b_hip_angle, r_b_hip_angle, l_f_limb_value, r_f_limb_value ,l_b_limb_value, r_b_limb_value):
        l_f_hip = Float64()
        l_f_hip.data = l_f_hip_angle
        r_f_hip = Float64()
        r_f_hip.data = r_f_hip_angle
        l_b_hip = Float64()
        l_b_hip.data = l_b_hip_angle
        r_b_hip = Float64()
        r_b_hip.data = r_b_hip_angle

        l_f_limb = Float64()
        l_f_limb.data = l_f_limb_value
        r_f_limb = Float64()
        r_f_limb.data = r_f_limb_value
        l_b_limb = Float64()
        l_b_limb.data = l_b_limb_value
        r_b_limb = Float64()
        r_b_limb.data = r_b_limb_value

        self.pub_l_f_hip_joint_position.publish(l_f_hip)
        self.pub_r_f_hip_joint_position.publish(r_f_hip)
        self.pub_l_b_hip_joint_position.publish(l_b_hip)
        self.pub_r_b_hip_joint_position.publish(r_b_hip)

        self.pub_l_f_limb_joint_position.publish(l_f_limb)
        self.pub_r_f_limb_joint_position.publish(r_f_limb)
        self.pub_l_b_limb_joint_position.publish(l_b_limb)
        self.pub_r_b_limb_joint_position.publish(r_b_limb)


    def insect_joints_callback(self, msg):
        """
        sensor_msgs/JointState
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

        :param msg:
        :return:
        """
        self.insect_joint_dictionary = dict(zip(msg.name, msg.position))


    def convert_angle_to_unitary(self, angle):
        """
        Removes complete revolutions from angle and converts to positive equivalent
        if the angle is negative
        :param angle: Has to be in radians
        :return:
        """
        # Convert to angle between [0,360)
        complete_rev = 2 * pi
        mod_angle = int(angle / complete_rev)
        clean_angle = angle - mod_angle * complete_rev
        # Convert Negative angles to their corresponding positive values
        if clean_angle < 0:
            clean_angle += 2 * pi

        return clean_angle

    def assertAlmostEqualAngles(self, x, y,):
        c2 = (sin(x) - sin(y)) ** 2 + (cos(x) - cos(y)) ** 2
        angle_diff = acos((2.0 - c2) / 2.0)
        return angle_diff

    def insect_check_continuous_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'base_waist_joint', 'body_head_joint', 'waist_body_joint is near the value given
        We have to convert the joint values removing whole revolutions and converting negative versions
        of the same angle
        :param joint_name:
        :param value:
        :param error: In radians
        :return:
        """
        joint_reading = self.insect_joint_dictionary.get(joint_name)
        if not joint_reading:
            print "self.insect_joint_dictionary="+str(self.insect_joint_dictionary)
            print "joint_name===>"+str(joint_name)
            assert "There is no data about that joint"
        clean_joint_reading = self.convert_angle_to_unitary(angle=joint_reading)
        clean_value = self.convert_angle_to_unitary(angle=value)

        dif_angles = self.assertAlmostEqualAngles(clean_joint_reading, clean_value)
        similar = dif_angles <= error

        return similar

    def insect_movement_look(self, l_f_hip_angle, r_f_hip_angle, l_b_hip_angle, r_b_hip_angle, l_f_limb_value ,r_f_limb_value,l_b_limb_value,r_b_limb_value):
        """
        Move:
        'torso_l_f_hip_joint',
        'torso_r_f_hip_joint',
        'torso_l_b_hip_joint',
        'torso_r_b_hip_joint',
        'l_f_hip_l_f_limb_joint',
        'r_f_hip_r_f_limb_joint',
        'l_b_hip_l_b_limb_joint'
        'r_b_hip_r_b_limb_joint'
        :return:
        """
        check_rate = 5.0
        position_l_f_hip = l_f_hip_angle
        position_r_f_hip = r_f_hip_angle
        position_l_b_hip = l_b_hip_angle
        position_r_b_hip = r_b_hip_angle

        position_l_f_limb = l_f_limb_value
        position_r_f_limb = r_f_limb_value
        position_l_b_limb = l_b_limb_value
        position_r_b_limb = r_b_limb_value

        similar_l_f_hip = False
        similar_r_f_hip = False
        similar_l_b_hip = False
        similar_r_b_hip = False

        similar_l_f_limb = False
        similar_r_f_limb = False
        similar_l_b_limb = False
        similar_r_b_limb = False

        rate = rospy.Rate(check_rate)
        while not (similar_l_f_hip and similar_r_f_hip and similar_l_b_hip and similar_r_b_hip and similar_l_f_limb and similar_r_f_limb and similar_l_b_limb and r_b_limb):
            self.move_insect_all_joints(position_l_f_hip,
                                       position_r_f_hip,
                                       position_l_b_hip,
                                       position_r_b_hip,
                                       position_l_f_limb,
                                       position_r_f_limb,
                                       position_l_b_limb,
                                       position_r_b_limb)
            similar_l_f_hip = self.insect_check_continuous_joint_value(joint_name="l_f_hip_joint",
                                                                         value=position_l_f_hip)
            similar_r_f_hip = self.insect_check_continuous_joint_value(joint_name="r_f_hip_joint",
                                                                         value=position_r_f_hip)
            similar_l_b_hip = self.insect_check_continuous_joint_value(joint_name="l_b_hip_joint",
                                                                         value=position_l_b_hip)
            similar_r_b_hip = self.insect_check_continuous_joint_value(joint_name="r_b_hip_joint",
                                                                         value=position_r_b_hip)
            similar_l_f_limb = self.insect_check_continuous_joint_value(joint_name="l_f_limb_joint",
                                                                         value=position_l_f_limb)
            similar_r_f_limb = self.insect_check_continuous_joint_value(joint_name="r_f_limb_joint",
                                                                         value=position_r_f_limb)
            similar_l_b_limb = self.insect_check_continuous_joint_value(joint_name="l_b_limb_joint",
                                                                         value=position_l_b_limb)
            similar_r_b_limb = self.insect_check_continuous_joint_value(joint_name="r_b_limb_joint",
                                                                         value=position_r_b_limb)


            rate.sleep()

    def insect_init_pos_sequence(self):
        """
        HIP limits lower="-pi/12" upper="pi/12"
        LIMB limits lower="-pi/12" upper="pi/12"
        :return:
        """
        l_f_hip_angle = -pi/12
        r_f_hip_angle = -pi/12
        l_b_hip_angle = -pi/12
        r_b_hip_angle = -pi/12
        l_f_limb_angle = pi/12
        r_f_limb_angle = pi/12
        l_b_limb_angle = pi/12
        r_b_limb_angle = pi/12
        self.insect_movement_look(l_f_hip_angle,
                                 r_f_hip_angle,
                                 l_b_hip_angle,
                                 r_b_hip_angle,
                                 l_f_limb_angle,LaserScanProcess
                                 r_f_limb_angle,
                                 l_b_limb_angle,
                                 r_b_limb_angle)

        l_f_limb_angle = -pi/12
        r_f_limb_angle = -pi/12
        l_b_limb_angle = -pi/12
        r_b_limb_angle = -pi/12
        self.insect_movement_look(l_f_hip_angle,
                                 r_f_hip_angle,
                                 l_b_hip_angle,
                                 r_b_hip_angle,
                                 l_f_limb_angle,
                                 r_f_limb_angle,
                                 l_b_limb_angle,
                                 r_b_limb_angle)

    def insect_hop(self, num_hops=15):
        """
        UPPER limits lower="-pi/12" upper="pi/12"
        LOWER limits lower="-pi/12" upper="pi/12"
        :return:
        """

        upper_delta = 1
        basic_angle = -pi/12
        angle_change = random.uniform(0.2, 0.7)
        hip_angle = basic_angle
        limb_angle = basic_angle - upper_delta * angle_change * 2.0

        #self.gurdy_init_pos_sequence()
        for repetitions in range(num_hops):
            self.insect_movement_look(l_f_hip_angle,
                                     r_f_hip_angle,
                                     l_b_hip_angle,
                                     r_b_hip_angle,
                                     l_f_limb_angle,
                                     r_f_limb_angle,
                                     l_b_limb_angle,
                                     r_b_hip_angle)

            upper_delta *= -1
            if upper_delta < 0:
                hip_angle = basic_angle + angle_change
            else:
                hip_angle = basic_angle
            limb_angle = basic_angle - upper_delta * angle_change * 2.0


    def insect_moverandomly(self):
        """
        UPPER limits lower="-pi/12" upper="pi/12"
        LOWER limits lower="-pi/12" upper="1.5708"
        :return:
        """
        l_f_hip_angle = random.uniform(-pi/12, pi/12)
        r_f_hip_angle = random.uniform(-pi/12, pi/12)
        l_b_hip_angle = random.uniform(-pi/12, pi/12)
        r_b_hip_angle = random.uniform(-pi/12, pi/12)
        l_f_limb_angle = random.uniform(-pi/12, pi/12)
        r_f_limb_angle = random.uniform(-pi/12, pi/12)
        l_b_limb_angle = random.uniform(-pi/12, pi/12)
        r_b_limb_angle = random.uniform(-pi/12, pi/12)
        self.insect_movement_look(l_f_hip_angle,
                                 r_f_hip_angle,
                                 l_b_hip_angle,
                                 r_b_limb_angle,
                                 l_f_limb_angle,
                                 r_f_limb_angle,
                                 l_b_limb_angle,
                                 r_b_limb_angle)

    def movement_random_loop(self):
        """
        Executed movements in a random way
        :return:
        """
        rospy.loginfo("Start Moving Insect...")
        while not rospy.is_shutdown():
            self.insect_init_pos_sequence()
            #self.insect_moverandomly()
            self.insect_hop()

if __name__ == "__main__":
    insect_jointmover_object = insectJointMover()
    insect_jointmover_object.movement_random_loop()
