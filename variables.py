import wpimath.geometry
import math

kMaxSpeed = 1.5  # meters per second
kRMaxSpeed = 0.1
kTMaxSpeed = 1.0
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

