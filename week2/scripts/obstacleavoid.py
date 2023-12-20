#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

msg=Twist()
case=''
linearx=0
angularz=0

# i followed a tutorial : https://github.com/SravanChittupalli/Obstacle-Avoidance-Bot-Using-ROS/blob/master/src/scripts/obstacle_avoidance.py


def callback(data):
    range={
        "right" : min(min(data.ranges[0:239]) , 2),
        "center" : min(min(data.ranges[240:479]) , 2),
        "left" : min(min(data.ranges[480:719]) , 2)
    }

    if ( range["right"] >1  and range["center"] > 1 and range["left"] >1):
        case = 'free'
        linearx=0.6
        angularz=0
    elif ( range["right"] > 1  and range["center"] < 1 and range["left"] > 1 ):
        case = 'front obstacle'
        linearx=0
        angularz=-0.5
    elif ( range["right"] < 1  and range["center"] > 1 and range["left"] > 1 ):
        case = 'right obstacle'
        linearx=0
        angularz=0.5
    elif ( range["right"] > 1  and range["center"] > 1 and range["left"] < 1 ):
        case = 'left obstacle'
        linearx=0
        angularz=-0.5
    

    print(case)

    rospy.loginfo(case)
    msg.linear.x = linearx
    msg.angular.z = angularz
    pub.publish(msg)

    

def listener():

    #assignment1
    global pub
    rospy.init_node('obstacle_avoid', anonymous=False)
    pub = rospy.Publisher("/cmd_vel" , Twist , queue_size=1)
    sub = rospy.Subscriber('/base_scan', LaserScan, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()