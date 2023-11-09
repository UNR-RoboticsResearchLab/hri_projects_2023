#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import JointState

def looker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('looker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        joint_states = JointState()
        joint_states.header.stamp = rospy.get_rostime()
        joint_states.header.frame_id="Torso"
        joint_states.name.append("HeadYaw")
        joint_states.name.append("HeadPitch")

        joint_states.position.append(0.17)
        joint_states.position.append(-0.39)
        pub.publish(joint_states)
        rate.sleep()

if __name__ == '__main__':
    try:
        looker()
    except rospy.ROSInterruptException:
        pass
