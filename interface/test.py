import RobotModel
import RobotSerial


if __name__ == '__main__':
    rm = RobotModel.RobotModel()
    ss = RobotSerial.SerialSession(rm, '/dev/ttyACM4')
    rm.speedLeft = -70
    rm.speedRight = -70
    # rm.clawClosed = True
    ss.updateRobotState(rm) # will update rm.reedSwitch
