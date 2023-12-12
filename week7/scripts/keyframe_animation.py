#!/usr/bin/python3
# license removed for brevity

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from tf import 

current_state = JointState()
keyframe_goals = []
keyframe_points = []
names = None


def input_keys():
    while(not rospy.shutdown()):
        print("adjust nao to key positions")
        keyframe_in = input("press s to save a position. press p to play animation. press q to exit.")
        
        if(keyframe_in=="s"):
            keyframe_goals.append(current_state)

        if(keyframe_in="p"):


def joint_state_callback(msg):
    global current_state, keyframe_goals
    current_state = msg

    names = msg.name

def interpolate(p1, p2):
    cnt = 0
    for states in p1.position:
        if (p1.position[cnt] == p1.position[cnt]):
            keyframe_points.append(0)
        else:
            np.linspace(p1.position[cnt])

            #append middle man with appropriate index 

def state():
    pub = rospy.Publisher('joint_states')
    sub = rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.init_node('keyframe_anim', anonymous=True)
    rate = rospy.Rate(10) # 10hz

if __name__ == '__main__':
    try:
        input_keys()
    except rospy.ROSInterruptException:
        pass