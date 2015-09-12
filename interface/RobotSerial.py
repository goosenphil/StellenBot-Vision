#!/usr/bin/env python2

import RobotModel
import serial

class SerialSession():
    def __init__(self, robotModel):
        self.session = serial.Serial('/dev/robot', 9600)
        self.currentRobotModel = RobotModel()
        self.updateRobotState(robotModel)

    def close(self):
        self.session.close()

    def __pollReedSwitchState(self):
        self.sendBytes(chr(3))
        r = self.session.read()
        if(r == chr(1)):
            return True
        else:
            return False

    def updateRobotState(self, robotModel):
        # update speed of left motor
        if (self.currentRobotModel.speedLeft != robotModel.speedLeft):
            if (robotModel.speedLeft < 0):
                self.sendBytes(chr(0), chr(1), chr(abs(robotModel.speedLeft)))
            else:
                self.sendBytes(chr(0), chr(0), chr(robotModel.speedLeft))

        # update speed of right motor
        if (self.currentRobotModel.speedRigth != robotModel.speedRight):
            if (robotModel.speedRight < 0):
                self.sendBytes(chr(1), chr(1), chr(abs(robotModel.speedRight)))
            else:
                self.sendBytes(chr(1), chr(0), chr(robotModel.speedRight))

        # close/open the claw
        if (self.currentRobotModel.clawClosed != robotModel.clawClosed):
            self.sendBytes(chr(2), chr(robotModel.clawClosed))

        # update the state of the reed switch
        robotModel.reedSwitch = self.__pollReedSwitchState()

        # set currentRobotModel to robotModel
        self.currentRobotModel = robotModel

    def sendBytes(self, byte1 = chr(0), byte2 = chr(0), byte3 = chr(0))
        self.session.write(chr(0xff) + chr(0xfe) + byte1 + byte2 + byte3)