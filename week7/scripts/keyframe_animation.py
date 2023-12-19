#!/usr/bin/python3
# license removed for brevity

import rospy
import numpy as np

from sensor_msgs.msg import JointState
import tf

current_state = JointState()
keyframe_goals = []
keyframe_points = []
names = None

pub=None

def input_keys():
    global pub
    while(not rospy.is_shutdown()):
        print("adjust nao to key positions")
        keyframe_in = input("move to position then press s to save a position. \npress g to generate animation\npress p to play animation. \npress q to exit.")
        
        if(keyframe_in=="s"):
            keyframe_goals.append(current_state)
            rospy.loginfo("joint state saved")

        if(keyframe_in=="g"):
            interpolate_list()

        if(keyframe_in=="p"):

            for state in keyframe_points:
                state.header.stamp=rospy.Time.now()
                pub.publish(state)
                rospy.sleep(0.01)

            print("insert play code here")

        if(keyframe_in=="q"):
            return


def joint_state_callback(msg):
    global current_state, keyframe_goals,names
    current_state = msg

    names = msg.name

def interpolate(p1, p2):
    slope_arr = []
    for i in range(len(p1.position)):
        if (p1.position[i] == p2.position[i]):
            slope_arr.append(0)
        else:

            slope_arr.append((p2.position[i]-p1.position[i])/100)

    print(slope_arr)

    return slope_arr
            #append middle man with appropriate index 

def add(a, b):
    print("A: "+ str(a))
    print(b)
    return a + b

def interpolate_list():
    global current_state, keyframe_goals, keyframe_points

    slope_arr_arr = []

    for i in range(len(keyframe_goals)-1):

        slope_arr_arr.append(interpolate(keyframe_goals[i], keyframe_goals[i+1]))

    print(slope_arr_arr)

    keyframe_points.append(keyframe_goals[0])

    for slp in slope_arr_arr:

        for i in range(100):
            joint_state = JointState()

            joint_state.header.frame_id=""
            joint_state.name=names
            joint_state.position=list(map(add, keyframe_points[i].position, slp))

            joint_state.velocity=[]
            joint_state.effort=[]
            keyframe_points.append(joint_state)

        
def init():
    global arm_initial_states, pub
    init_joint_state = JointState()
    init_joint_state.header.stamp = rospy.Time.now()
    init_joint_state.header.frame_id = ""
    #init_joint_state.name = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"]
    init_joint_state.name = ["HeadYaw","HeadPitch","LHipYawPitch","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll","RHipYawPitch",
                             "RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll","LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","LWristYaw","LHand","RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll","RWristYaw","RHand","RFinger23","RFinger13","RFinger12","LFinger21","LFinger13","LFinger11","RFinger22","LFinger22","RFinger21","LFinger12","RFinger11","LFinger23","LThumb1","RThumb1","RThumb2","LThumb2"]
    
    init_joint_state.position = [0.0, -9.093359999989836e-05, -0.00010594240000005861, -0.0001550409999999669, -0.00038482599999989375, 
                                 -0.00016400378000000493, -0.6278536161, 0.23566920370000005, -0.00010594240000005861, -0.0001959274999999705, -0.00038482599999989375, -0.00016400378000000493, -4.8639999999711137e-05, -0.0001017730000000272, -0.15600811599999997, -0.00014643740000003236, 0.0, -0.7897633000000001, 0.0, 0.0, 0.5372685920000002, -1.0856085988, 2.08567, 0.60361563778, 0.33851027199999995, 0.7452, 0.7451247348, 0.7451247348, 0.7451247348, 0.0, 0.0, 0.0, 0.7451247348, 0.0, 0.7451247348, 0.0, 0.7451247348, 0.0, 0.0, 0.7451247348, 0.7451247348, 0.0]
    
    init_joint_state.velocity = []
    init_joint_state.effort = []
    
    rospy.loginfo("initializing robot to initial state")
    
    rospy.loginfo(init_joint_state)
    
    pub.publish(init_joint_state)


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('joint_states',JointState, queue_size=10)
        sub = rospy.Subscriber('/joint_states', JointState, joint_state_callback)
        rospy.init_node('keyframe_anim', anonymous=True)
        rate = rospy.Rate(10) # 10hz

        init()
        input_keys()
    except rospy.ROSInterruptException:
        pass