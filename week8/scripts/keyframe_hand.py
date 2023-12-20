#!/usr/bin/python3

import rospy
import numpy as np

from sensor_msgs.msg import JointState
import tf2_ros
import math

current_state = JointState()
names = None

pub=None


# worked on this with josh and tyler

def get_hand_tf():
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    tf_hand = tf_buffer.lookup_transform('Neck', 'l_gripper', rospy.Time(), rospy.Duration(1.0))

    return tf_hand

def hand_callback(msg):
    global current_state
    current_state=msg

def main_look():
    global current_state

    while not rospy.is_shutdown():

        joint_state = JointState()
        joint_state.header.frame_id="base_link"
        
        joint_state.name.append("HeadYaw")
        joint_state.name.append("HeadPitch")
        
        transform = get_hand_tf()

        h_yaw = current_state.position[0]
        h_pitch = current_state.position[1]

        g_yaw = math.atan2(transform.transform.translation.y, transform.transform.translation.x)
        g_pitch = -math.atan2(transform.transform.translation.z, transform.transform.translation.x)

        joint_state.header.stamp = rospy.Time.now()

        joint_state.position.append(h_yaw + g_yaw)
        joint_state.position.append(g_pitch)

        pub.publish(joint_state)


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/animation',JointState, queue_size=10)
        sub = rospy.Subscriber('/joint_states', JointState, hand_callback)
        rospy.init_node('keyframe_anim', anonymous=True)
        rate = rospy.Rate(10) # 10hz

        main_look()
    except rospy.ROSInterruptException:
        pass