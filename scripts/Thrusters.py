#!/usr/bin/env python3
'''
Subscribe Topics:

inserr/thruster/power
    messsage: FL, FR, BL, BR, UF, UB <Vector6: -1, 1>

Publish Topics:

inserr/can/thruster
    address <hexadecimal>
    data <bytearray>

thurster.info
    "thrusters_output": FL, FR, BL, BR, BL, UF, UB <32767, -32768> Integer

'''


import rospy
import pubsub.json_message
import numpy as np

class Thrusters:
    def __init__ (self, step_multiplier, rate):
        self.Thrusters = rospy.get_param('thrusters_config', [])
        print(self.Thrusters)

        self.thruster_addresses = list(map(lambda x: x["Address"], self.Thrusters))
        self.thruster_deadzones = list(map(lambda x: x["Deadzone"], self.Thrusters))

        #directly to can handler
        self.pub_can = pubsub.json_message.Publisher("inserr/can/thruster", queue_size=10)
        self.pub_thruster = pubsub.json_message.Publisher("inserr/thruster/info", queue_size=10)
        pubsub.json_message.Subscriber("inserr/thruster/power", self.listener)

        self.step = step_multiplier * (1/rate)
        self.current_power = [0,0,0,0,0,0]
        self.output_power = [0,0,0,0,0,0]
        self.target_power = [0, 0, 0, 0, 0, 0]
        
    def apply_deadzone(self, power, deadzone):
        if power > 0:
            return self.valmap(power, 0, 1, deadzone, 1)
        elif power < 0:
            return self.valmap(power, 0, -1, -deadzone, -1)
        else:
            return 0

    def calculate_gradual_output(self, index):
        #cache currentpower variable
        difference = self.target_power[index] - self.current_power[index]
        if abs(difference) > self.step:
            self.current_power[index] += difference/abs(difference)*self.step
        else:
            self.current_power[index] = self.target_power[index]

        out_power = self.current_power[index]
        if abs(out_power) > 1:
            out_power = out_power/abs(out_power) 

        return int(out_power*32767) if out_power>=0 else int(out_power*32768)


    @staticmethod
    def valmap(value, istart, istop, ostart, ostop):
      return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

    def send_can(self, address, power):
        self.pub_can.publish({"type": "CAN", "address": address, "data": [32, power >> 8 & 0xff, power & 0xff]})

    def listener(self, message):
        rospy.loginfo(self.thruster_deadzones)
        for index in range(len(self.target_power)):
            self.target_power[index] = self.apply_deadzone(message[index], self.thruster_deadzones[index])

    def run(self):
        for index in range(len(self.target_power)):
            self.output_power[index] = self.calculate_gradual_output(index)
            self.send_can(self.thruster_addresses[index], self.output_power[index])
        self.pub_thruster.publish({"outputs": self.output_power})
            

if __name__ == '__main__':
    try:
        rospy.init_node('thrusters', anonymous=True)
        rate_param = rospy.get_param('~rate', 10.0)
        rate = rospy.Rate(rate_param)
        
        thrusters = Thrusters(5, rate_param)

        while not rospy.is_shutdown():
            thrusters.run()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass