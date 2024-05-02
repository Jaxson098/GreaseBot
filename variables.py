import wpimath.geometry
import math
import robot

#these are global variables:

#the max speed of the robot
kMaxSpeed = 4.0  # meters per second
# kRMaxSpeed = 4.0 #NOTE: can we get of this it is note used
kTMaxSpeed = 2.0 #NOTE: should thi bnot be for, it is used to limit y speed
kMaxAngularSpeed = math.pi  # 1/2 rotation per second

#NOTE: cna we get rid of these non of them are used
# frontLeftZero = 0
# frontRightZero = 0
# backLeftZero = 0
# backRightZero = 0
# zeroThreshold = wpimath.geometry.Rotation2d(0.3)
# drivePID_P = 0.02
# drivePID_I = 0
# drivePID_D = 0

#PID values for turn motor
turnPID_P = 0.9 # the inportant one
turnPID_I = 0
turnPID_D = 0

#voltadge required to voercome friction of the ground to start moveing for drive motor
driveFF_1 = 0.7
#voltadge required to maintain a constant velocity for for drive motor
driveFF_2 = 3
#voltadge required to voercome friction of the ground to start moveing for turn motor
turnFF_1 = 0.3 #0.05
#voltadge required to maintain a constant velocity for turn motor
turnFF_2 = 1 #0.45 

# TurnState = 0 #NOTE: can we get rid of this it is not used