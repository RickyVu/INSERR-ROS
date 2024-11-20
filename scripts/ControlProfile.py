#!/usr/bin/env python3
import sys
import rospy
import pubsub.json_message

class ControlProfile:
    def __init__(self, max_percentage = 100, formula_modifier = 30, activate = 'A'):
        self.debug = pubsub.json_message.Publisher('inserr/control/debug', queue_size=10)
        self.movement_pub = pubsub.json_message.Publisher('inserr/control/movement', queue_size=10)
        pubsub.json_message.Subscriber('inserr/input/movement', self.movement_listener)
        pubsub.json_message.Subscriber('inserr/input/profile', self.profile_listener)
        self.max_percentage = int(max_percentage)/100
        self.formula_modifier = float(formula_modifier)
        self.activate = activate
        self.profile_change = 'A'

    @staticmethod
    def power_function(A, B):
        if A >=0:
            return 1/B*(((B+1)**A)-1)
        else:
            return -1/B*(((B+1)**-A)-1)

    def movement_listener(self, message):
        if self.profile_change == self.activate:
            Strafe, Drive, Yaw, Updown, TiltFB, TiltLR = message["gamepad_movement"]
            Strafe = self.power_function(Strafe, self.formula_modifier)
            Drive = self.power_function(Drive, self.formula_modifier)
            Yaw = self.power_function(Yaw, self.formula_modifier)
            Updown = self.power_function(Updown, self.formula_modifier)
            TiltFB = self.power_function(TiltFB, self.formula_modifier)
            TiltLR = self.power_function(TiltLR, self.formula_modifier)

            Strafe *= self.max_percentage
            Drive *= self.max_percentage
            Yaw *= self.max_percentage
            Updown *= self.max_percentage
            TiltFB *= self.max_percentage
            TiltLR *= self.max_percentage

            self.movement_pub.publish([Strafe, Drive, Yaw, Updown, TiltFB, TiltLR])
            self.debug.publish([rospy.get_name(), self.max_percentage, self.formula_modifier, self.activate])

    def profile_listener(self, message):
        self.profile_change = message["gamepad_profile"]

if __name__ == '__main__':
    if len(sys.argv)<2:
        raise SystemError
    try:
        rospy.init_node('ControlProfile', anonymous=True)
        #param_dir = sys.argv[1]
        node_name = rospy.get_name()
        max_percentage_param = rospy.get_param(f'{node_name}/max_percentage', 100)
        formula_modifier_param = rospy.get_param(f'{node_name}/formula_modifier', 30)
        activate_param = rospy.get_param(f'{node_name}/activate', "A")
        ControlProfile(max_percentage_param, formula_modifier_param, activate_param)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass