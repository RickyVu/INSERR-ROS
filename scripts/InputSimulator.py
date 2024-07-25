#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import pubsub.json_message
import random

def input_simulator():
    pub_thruster = pubsub.json_message.Publisher('inserr/input/movement', queue_size=10)
    rospy.init_node('input_simulator', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        message = {"gamepad_movement": [random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)]}
        pub_thruster.publish(message)
        #rospy.loginfo(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        input_simulator()
    except rospy.ROSInterruptException:
        pass
