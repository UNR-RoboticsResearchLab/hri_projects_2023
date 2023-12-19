#!/usr/bin/python3
# license removed for brevity

import rospy

from std_msgs.msg import String

message = ""


pub = rospy.Publisher('tts/phrase', String, queue_size=10)

def listen(msg):
    global message
    print("Listened to:" + msg.data)
    message=msg.data



if __name__ == '__main__':
    try:
        rospy.init_node('listen_repeat', anonymous=True)
        sub = rospy.Subscriber('/speech_recognition/final_result', String, listen)
        rate = rospy.Rate(10) # 10hz

        pub.publish("is it raining today?")

        is_raining = False

        has_umbrella = False

        rospy.sleep(1)
        while(message!="yes" or message!='no'):
            rospy.sleep(1)

            if(message=="yes"):
                is_raining=True
                pub.publish("do you have an umbrella?")

                if(message=="yes"):
                    has_umbrella=True
                    pub.publish("good! i hope you stay dry today")
                    break
            
            elif(message=="no"):
                is_raining==False
                pub.publish("i hope it is a good day outside then")
                rospy.sleep(1)

            else:
                print("waiting")
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass