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
        case = 'weeeeeeee!'
        linearx=0.6
        angularz=0
    elif ( range["right"] > 1  and range["center"] < 1 and range["left"] > 1 ):
        case = 'something do be in front of me!'
        linearx=0
        angularz=-0.5
    elif ( range["right"] < 1  and range["center"] > 1 and range["left"] > 1 ):
        case = 'to the right!'
        linearx=0
        angularz=0.5
    elif ( range["right"] > 1  and range["center"] > 1 and range["left"] < 1 ):
        case = 'to the left!'
        linearx=0
        angularz=-0.5
    

    print(case)

    rospy.loginfo(case)
    msg.linear.x = linearx
    msg.angular.z = angularz
    pub.publish(msg)

    

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.


    global pub
    rospy.init_node('obstacle_avoid', anonymous=False)
    pub = rospy.Publisher("/cmd_vel" , Twist , queue_size=1)
    sub = rospy.Subscriber('/base_scan', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()