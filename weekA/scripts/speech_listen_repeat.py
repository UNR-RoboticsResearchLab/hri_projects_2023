#!/usr/bin/python3
# license removed for brevity

import rospy
import ros_vosk

from std_msgs.msg import String

pub = rospy.Publisher('tts/phrase', String, queue_size=10)

def listen(msg):
    print("Listened to:" + msg.data)

    pub.publish(msg.data)


if __name__ == '__main__':
    try:
        rospy.init_node('listen_repeat', anonymous=True)
        sub = rospy.Subscriber('/speech_recognition/final_result', String, listen)
        rate = rospy.Rate(10) # 10hz
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass