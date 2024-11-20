#!/usr/bin/env python3
'''
Subcribe Topics:

inserr/control/movement
    message: Strafe, Drive, Yaw, UpDown, TiltFB, TiltLR <Vector6: -1, 1>

Publish Topics:

inserr/thruster/power
    messsage: FL, FR, BL, BR, UF, UB <Vector6: -1, 1>
'''

import numpy as np
import time

import rospy
import pubsub.json_message


#Scale constants
SCALE_CONSTANTS = [1,1,1,1,1,1,1]
DEFAULT_CG = [0, 0, 0]

class ThrusterPower:
    def __init__ (self):
        self.CG = np.array(rospy.get_param('~CG', DEFAULT_CG))
        self.Thrusters = rospy.get_param('thrusters_config', [])
        
        self.pub_thruster = pubsub.json_message.Publisher('inserr/thruster/power', queue_size=10)
        pubsub.json_message.Subscriber('inserr/control/movement', self.command_movement)


        self.ThrusterMatrix = np.zeros((6,1))
        self.num_thrusters = len(self.Thrusters)
        self.counter = 0
        self.finalList = None
        self.thruster_scaling = [1] * self.num_thrusters

        for i, Thruster in enumerate(self.Thrusters): # 6x6 Matrix
            ThrusterPosition = Thruster["Position"]
            ThrusterDirection = Thruster["Direction"]
            ThrusterPosition = np.subtract(ThrusterPosition, self.CG)                                   #1, 3
            Torque = np.cross(ThrusterPosition, ThrusterDirection)                                      #1, 3
            ThrusterArray = np.concatenate((ThrusterDirection, Torque)).reshape(6,1)                    #1, 6
            self.ThrusterMatrix = np.concatenate((self.ThrusterMatrix, ThrusterArray), axis = 1)
        
        for i in range(6):
            message = [0] * 6
            message[i] = -1
            self.gamepadScaleConstant(message)
        self.gamepadScaleConstant([0,0,0,1,0,0])

    def truncate(self, finalList):
        if max(abs(finalList)) > 1:
            for counter, Thruster in enumerate(self.Thrusters):
                finalList[counter, 0] /= max(abs(finalList))
        return finalList

    def directionScale(self, finalList):
        for counter, Thruster in enumerate(self.Thrusters):
            if finalList[counter, 0] < 0:
                finalList[counter, 0] *= Thruster["NegativeScale"]
            else:
                finalList[counter, 0] *= Thruster["PositiveScale"]
            #finalList[counter,0] /= Thruster["Scale"] # uncomment for combinational movement
            if Thruster["Invert"] == True:
                finalList[counter, 0] *= -1
        return finalList

    def invert(self, finalList):
        for counter, Thruster in enumerate(self.Thrusters):
            finalList[counter, 0] *= -1
        return finalList

    def overallScale(self, finalList):
        for counter, Thruster in enumerate(self.Thruster):
            finalList[counter, 0] /= Thruster["Scale"]

    def pseudoInv(self, expectedResult):
        ThrusterMatrixInv = np.linalg.pinv(self.ThrusterMatrix[0:6,1:7])
        finalList = ThrusterMatrixInv.dot(expectedResult)
        return finalList

    def gamepadScale(self, message):
        gamepadScaled = list(message)
        for counter, dof in enumerate(gamepadScaled):
            if counter == 3 and dof > 0:
                gamepadScaled[counter] *= SCALE_CONSTANTS[6]
            else:
                gamepadScaled[counter] *= SCALE_CONSTANTS[counter]
        return gamepadScaled

    def gamepadScaleConstant(self, message):
        Strafe, Drive, Yaw, Updown, TiltFB, TiltLR = message
        expectedResult = np.array((Strafe, Drive, Updown, TiltFB, TiltLR, Yaw)).reshape(6,1)
        finalList = self.pseudoInv(expectedResult)
        finalList = self.directionScale(finalList)
        for counter, dof in enumerate(message):
            if dof != 0 and max(abs(finalList)) != 0:
                SCALE_CONSTANTS[self.counter] = float(1/max(abs(finalList)))
        self.counter += 1

    def command_movement(self, message):

        message = self.gamepadScale(message)
        Strafe, Drive, Yaw, Updown, TiltFB, TiltLR = message
        expectedResult = np.array((Strafe, Drive, Updown, TiltFB, TiltLR, Yaw)).reshape(6,1)
        finalList = self.pseudoInv(expectedResult)
        finalList = self.directionScale(finalList)
        finalList = self.invert(finalList)
        ############ inverted from thruster desired direction to thrust direction
        finalList = self.truncate(finalList)

        finalList = finalList.reshape(1,6)
        finalList = finalList.tolist()
        self.finalList = [item for item in finalList if isinstance(item,list)]
        

    def run(self):
        if self.finalList is not None:
            self.pub_thruster.publish(self.finalList[0])


if __name__ == "__main__":
    try:
        rospy.init_node('thruster_power', anonymous=True)
        rate_param = rospy.get_param('~rate', 60.0)
        rate = rospy.Rate(rate_param)
        
        thruster_power = ThrusterPower()

        while not rospy.is_shutdown():
            thruster_power.run()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass