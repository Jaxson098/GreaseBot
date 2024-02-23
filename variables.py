# Global Vars

import wpimath.geometry
import math
import robot

kMaxSpeed = 2.0  # meters per second
kRMaxSpeed = 0.1
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
turnPID_P = 0.04
turnPID_I = 0
turnPID_D = 0.000053
driveFF_1 = 1
driveFF_2 = 3
turnFF_1 = 0
turnFF_2 = 0.45
TurnState = 0
IntakeMotor1ID = 16
IntakeMotor2ID = 17
shootMotor1ID = 14
shootMotor2ID = 15
self.shootMotor1ID.set(1)
self.shootMotor2ID.set(2)
self.shootMotor1ID.set(5)
self.shootMotor2ID.set(5) 
self.shootMotor1ID.set(0)
self.shootMotor2ID.set(0)
self.IntakeMotor1ID.set(-2)
self.IntakeMotor2ID.set(-3) 
self.IntakeMotor1ID.set(0)
self.IntakeMotor2ID.set(0)
  
def setTurnState(rot) -> None:
    global TurnState
    if abs(rot) > 0:
        TurnState = 1
    else:
        TurnState = 0
    #print(TurnState)
    #return TurnState
