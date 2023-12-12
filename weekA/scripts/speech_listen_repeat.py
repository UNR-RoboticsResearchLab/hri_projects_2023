#!/usr/bin/python3
# license removed for brevity

import rospy
import ros_vosk

from speech_recognition.msg import speech_recognition

current_word = speech_recognition()

def listen(msg):
    global current_word
    current_word = msg

def driver():
    pub = rospy.Publisher('/speech_recognition/final_result')
    sub = rospy.Subscriber('/speech_recognition/vosk_result', speech_recognition, listen)
    rospy.init_node('listen_repeat', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    pub.publish(current_word)


if __name__ == '__main__':
    try:
        driver()
    except rospy.ROSInterruptException:
        pass