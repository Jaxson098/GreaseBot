import ntcore
import math
import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import drivetrain
import variables
from wpimath.kinematics import SwerveModuleState
import wpiutil
from wpiutil import wpistruct
import dataclasses
from wpilib import Timer

#custom data structure to send encoder data via network tables to advantage scope
@wpiutil.wpistruct.make_wpistruct(name="CANCoders")
@dataclasses.dataclass

#encoder data sent is relitive not apsalute since they start on zero
class CANCodersPosition:
    frontLeftTurn:wpistruct.double = 0.0
    frontRightTurn:wpistruct.double = 0.0
    backLeftTurn:wpistruct.double = 0.0
    backRightTurn:wpistruct.double = 0.0

#custom data structure to send drive data via network tables to advantage scope
@wpiutil.wpistruct.make_wpistruct(name="Drive")
@dataclasses.dataclass

#drive data sent starts at zero
class Drive:
    xSpeed:wpistruct.double = 0.0
    ySpeed:wpistruct.double = 0.0
    rot:wpistruct.double = 0.0

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""

        #defines controler as a xbox controler on port 0
        self.controller = wpilib.XboxController(0)
        #defines swerve as the drivetrain module
        self.swerve = drivetrain.Drivetrain()

        #makes the slewrate limiters
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        #makes and starts a timer to be used for logging intervals
        self.timer = Timer()
        self.timer.start()

        # logging to NT4
        nt = ntcore.NetworkTableInstance.getDefault()
        topicSS = nt.getStructArrayTopic("/SwerveStatesAdrian", SwerveModuleState)
        self.pubSS = topicSS.publish()

        topicCE = nt.getStructTopic("/CANEncoders", CANCodersPosition)
        self.pubCE = topicCE.publish()

        topicDrive = nt.getStructTopic("/Dive", Drive)
        self.pubDrive = topicDrive.publish()

        # Align the wheels
        self.swerve.alignment()

    def logSwerveStates(self):
        #get states (vel and angle)
        frontLeftState = self.swerve.frontLeft.getState()
        frontRightState = self.swerve.frontRight.getState()
        backLeftState = self.swerve.backLeft.getState()
        backRightState = self.swerve.backRight.getState()
        # publish states to network table 
        self.pubSS.set([frontLeftState,frontRightState,backLeftState,backRightState])

    def logCANEncoders(self):
        #get cancoder pos
        canPost = CANCodersPosition()
        canPost.frontLeftTurn = self.swerve.frontLeft.turningEncoder.getAbsolutePosition()
        canPost.frontRightTurn = self.swerve.frontRight.turningEncoder.getAbsolutePosition()
        canPost.backLeftTurn = self.swerve.backLeft.turningEncoder.getAbsolutePosition()
        canPost.backRightTurn = self.swerve.backRight.turningEncoder.getAbsolutePosition()
        #publish pos
        self.pubCE.set(canPost)

    def autonomousInit(self) -> None:
        #no tested auto yet
        pass
        # self.autonomousTimer = Timer()
        # self.autonomousTimer.start()

    def autonomousPeriodic(self) -> None:
        #no tested auto yet
        pass
        # self.swerve.updateOdometry()
        # if self.autonomousTimer.get() < 2.0:
        #     pass
        #     xSpeed = 0.5
        #     ySpeed = 0
        #     rot = 0
        #     fieldRelative = False
        #     self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

    def logDrive(self,xSpeed, ySpeed, rot):
        #log desired motion
        drive=Drive
        drive.xSpeed=xSpeed
        drive.ySpeed=ySpeed
        drive.rot=rot
        #publish
        self.pubDrive.set(drive)

    def teleopPeriodic(self) -> None:
        #every 1 second log and publish data
        if self.timer.hasElapsed(1):
            self.logSwerveStates()
            self.logCANEncoders()
            self.timer.restart()
        #false = useing xspeed yspeed and rot from controler
        self.driveWithJoystick(False)

        if self.controller.getRawButton(1) == 1:
            #button 1 = auto align
            print("aligning")
            self.swerve.alignment()
        
        #NOTE: not sure if this is needed since you are just sending 0 power to wheels
        # if self.controller.getRawButton(4) == 1: # VAR
        #     self.swerve.drive(0,0,0,0,self.getPeriod())
        #     self.swerve.alignment()

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        #inverting everything since xbox and logitech controlers invert

        #xspeed = forward back on left y
        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftY(), 0.02)
            )
            * variables.kMaxSpeed
        )
        
        #yspeed = strafe left right on left x
        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftX(), 0.02) # VAR
            )
            * variables.kTMaxSpeed
        )

        #rot = turning on right x
        rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRightX(), 0.02)
            )
            * variables.kMaxSpeed
        )

        #useing xseed yspeed and rot log and publish
        self.logDrive(xSpeed=xSpeed,ySpeed=ySpeed,rot=rot)
        
        #drive
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())