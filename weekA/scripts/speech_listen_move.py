#!/usr/bin/python3
# license removed for brevity

import rospy

from std_msgs.msg import String
from sensor_msgs.msg import JointState

pub = rospy.Publisher('tts/phrase', String, queue_size=10)
nao_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

message=""

def listen(msg):
    global message
    print("Listened to:" + msg.data)

    message=msg.data

if __name__ == '__main__':
    try:
        rospy.init_node('listen_repeat', anonymous=True)
        sub = rospy.Subscriber('/speech_recognition/final_result', String, listen)
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            if(message=='wave'):
                init_joint_state = JointState()
                init_joint_state.header.stamp = rospy.Time.now()
                init_joint_state.header.frame_id = ""
                init_joint_state.name = ["HeadYaw","HeadPitch","LHipYawPitch","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll","RHipYawPitch",
                                        "RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll","LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","LWristYaw","LHand","RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll","RWristYaw","RHand","RFinger23","RFinger13","RFinger12","LFinger21","LFinger13","LFinger11","RFinger22","LFinger22","RFinger21","LFinger12","RFinger11","LFinger23","LThumb1","RThumb1","RThumb2","LThumb2"]
                
                init_joint_state.position =  [0.0, -9.093359999989836e-05, -0.00010594240000005861, -0.0001550409999999669, -0.00038482599999989375, -0.00016400378000000493, -0.00016097489999999937, -0.0017650318000000387, -0.00010594240000005861, -0.0001959274999999705, -0.00038482599999989375, -0.00016400378000000493, -4.8639999999711137e-05, -0.0001017730000000272, -1.5196191620000001, 0.7220496444, 0.0, -0.7897633000000001, 0.0, 0.0, 0.0, -1.7623500000008008e-05, 0.0, 0.7897633000000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


                
                init_joint_state.velocity = []
                init_joint_state.effort = []
                
                
                nao_pub.publish(init_joint_state)

            else:
                init_joint_state = JointState()
                init_joint_state.header.stamp = rospy.Time.now()
                init_joint_state.header.frame_id = ""
                init_joint_state.name = ["HeadYaw","HeadPitch","LHipYawPitch","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll","RHipYawPitch",
                                        "RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll","LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","LWristYaw","LHand","RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll","RWristYaw","RHand","RFinger23","RFinger13","RFinger12","LFinger21","LFinger13","LFinger11","RFinger22","LFinger22","RFinger21","LFinger12","RFinger11","LFinger23","LThumb1","RThumb1","RThumb2","LThumb2"]
                
                init_joint_state.position =  [0.0, -9.093359999989836e-05, -0.00010594240000005861, -0.0001550409999999669, -0.00038482599999989375, -0.00016400378000000493, -0.6278536161, 0.23566920370000005, -0.00010594240000005861, -0.0001959274999999705, -0.00038482599999989375, -0.00016400378000000493, -4.8639999999711137e-05, -0.0001017730000000272, -0.15600811599999997, -0.00014643740000003236, 0.0, -0.7897633000000001, 0.0, 0.0, 0.5372685920000002, -1.0856085988, 2.08567, 0.60361563778, 0.33851027199999995, 0.7452, 0.7451247348, 0.7451247348, 0.7451247348, 0.0, 0.0, 0.0, 0.7451247348, 0.0, 0.7451247348, 0.0, 0.7451247348, 0.0, 0.0, 0.7451247348, 0.7451247348, 0.0]

                
                init_joint_state.velocity = []
                init_joint_state.effort = []
                
                
                nao_pub.publish(init_joint_state)

        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass