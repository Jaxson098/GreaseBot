# Global Vars

import wpimath.geometry
import math
import robot

kMaxSpeed = 4.0  # meters per second
kRMaxSpeed = 4.0
kTMaxSpeed = 2.0
kMaxAngularSpeed = math.pi  # 1/2 rotation per second
frontLeftZero = 0
frontRightZero = 0
backLeftZero = 0
backRightZero = 0
zeroThreshold = wpimath.geometry.Rotation2d(0.3)
drivePID_P = 0.02
drivePID_I = 0
drivePID_D = 0

turnPID_P = 0.9 # the inportant one

turnPID_I = 0
turnPID_D = 0 #0.000053
driveFF_1 = 0.7
driveFF_2 = 3
turnFF_1 = 0.3 #0.05
turnFF_2 = 1 #0.45 
TurnState = 0

def setTurnState(rot) -> None:
    global TurnState
    if abs(rot) > 0:
        TurnState = 1
    else:
        TurnState = 0
    #print(TurnState)
    #return TurnState
