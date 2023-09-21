#!/usr/bin/python3
# license removed for brevity

import rospy
import random
from geometry_msgs.msg import Twist

def wiggler():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('wiggler', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    random.seed()
    while not rospy.is_shutdown():
        # generate random x and y values
        xvel = random.gauss(0, 0.3)
        yvel = random.gauss(0, 0.2)
        turn = 0 #random.gauss(0, 0.2)
        
        cmd_vel = Twist()
        cmd_vel.linear.x = xvel
        cmd_vel.linear.y = yvel
        cmd_vel.angular.z = turn

        pub.publish(cmd_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        wiggler()
    except rospy.ROSInterruptException:
        pass