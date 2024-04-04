#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import ntcore
import math
import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import drivetrain
import variables
# import shooter
from wpimath.kinematics import SwerveModuleState
import wpiutil
from wpiutil import wpistruct
import dataclasses
from wpilib import Timer
import arm

@wpiutil.wpistruct.make_wpistruct(name="CANCoders")
@dataclasses.dataclass

class CANCodersPosition:
    frontLeftTurn:wpistruct.double = 0.0
    frontRightTurn:wpistruct.double = 0.0
    backLeftTurn:wpistruct.double = 0.0
    backRightTurn:wpistruct.double = 0.0

@wpiutil.wpistruct.make_wpistruct(name="Drive")
@dataclasses.dataclass

class Drive:
    xSpeed:wpistruct.double = 0.0
    ySpeed:wpistruct.double = 0.0
    rot:wpistruct.double = 0.0

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.liftDirection = False
        self.shooterDirection = False
        self.arm = arm.Arm

        # self.controller = wpilib.Joystick(2) # VAR
        # self.controller = wpilib.XboxController(0) # VAR
        self.swerve = drivetrain.Drivetrain()
        # self.shooter = shooter.ShootModule()
        # navxGyro is a file to test the navx Gyro. This can be ignored/commented out.
        # self.navxGyro = navxGyro.Gyro()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        # Speed limiters

        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3) # VAR
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3) # VAR
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3) # VAR

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


        # Align the wheels to 0
        self.swerve.alignment()

    

    def logSwerveStates(self):
        frontLeftState = self.swerve.frontLeft.getState()
        frontRightState = self.swerve.frontRight.getState()
        backLeftState = self.swerve.backLeft.getState()
        backRightState = self.swerve.backRight.getState()
        # publish event to network table 
        self.pubSS.set([frontLeftState,frontRightState,backLeftState,backRightState])

    def logCANEncoders(self):
        canPost = CANCodersPosition()
        canPost.frontLeftTurn = self.swerve.frontLeft.turningEncoder.getAbsolutePosition()
        canPost.frontRightTurn = self.swerve.frontRight.turningEncoder.getAbsolutePosition()
        canPost.backLeftTurn = self.swerve.backLeft.turningEncoder.getAbsolutePosition()
        canPost.backRightTurn = self.swerve.backRight.turningEncoder.getAbsolutePosition()
        self.pubCE.set(canPost)


    def autonomousInit(self) -> None:
        self.autonomousTimer = Timer()
        self.autonomousTimer.start()

    #FUTURE
    def autonomousPeriodic(self) -> None:
        #self.driveWithJoystick(False)
        self.swerve.updateOdometry()
        if self.autonomousTimer.get() < 2.0:
            pass
            xSpeed = 0.5
            ySpeed = 0
            rot = 0
            fieldRelative = False
            self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
        

    def logDrive(self,xSpeed, ySpeed, rot):
        drive=Drive
        drive.xSpeed=xSpeed
        drive.ySpeed=ySpeed
        drive.rot=rot
        self.pubDrive.set(drive)


    def teleopPeriodic(self) -> None:
        # we want to publish data only every 1s, that's give us enough of data in Advantage Scope
        if self.timer.hasElapsed(1):
            self.logSwerveStates()
            self.logCANEncoders()
            self.timer.restart()
        
        # This one is an interesting one because Swerve example had the opposite - field relative true for teleop
        self.driveWithJoystick(False)

        # self.navxGyro.getGyro()
        #self.shootWithJoystick(False)
        #self.shooter.speakershootmotor(1, 1)
        #  xbox 1a 2b 4x 4y 5 left bumper 6right bumper 7 back 8 start 9 left stick 10 right stick press
        if self.controller.getRawButton(1) == 1: # VAR
            print("square button pressed")
            self.swerve.alignment()
        
        if self.controller.getRawButton(4) == 1: # VAR
            self.swerve.drive(0,0,0,0,self.getPeriod())
            self.swerve.alignment()

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        # NOTE: Check if we need inversion here
        # xSpeed = (
        #     -self.xspeedLimiter.calculate(
        #         # wpimath.applyDeadband(self.controller.getRawAxis(1), 0.1) # VAR
        #         wpimath.applyDeadband(self.controller.getLeftY(), 0.02) # VAR
        #     )
        #     * variables.kMaxSpeed
        # )

        # # Get the y speed or sideways/strafe speed. We are inverting this because
        # # we want a positive value when we pull to the left. Xbox controllers
        # # return positive values when you pull to the right by default.
        # # NOTE: Check if we need inversion here
        # ySpeed = (
        #     -self.yspeedLimiter.calculate(
        #         # wpimath.applyDeadband(self.controller.getRawAxis(2), 0.6) # VAR
        #         wpimath.applyDeadband(self.controller.getLeftX(), 0.02) # VAR
        #     )
        #     * variables.kTMaxSpeed
        # )

        # # Get the rate of angular rotation. We are inverting this because we want a
        # # positive value when we pull to the left (remember, CCW is positive in
        # # mathematics). Xbox controllers return positive values when you pull to
        # # the right by default.
        # # rot = (
        # #     (-self.rotLimiter.calculate(
        # #         wpimath.applyDeadband(self.controller.getRawAxis(3), 0.2) # VAR
        # #     )
        # #     * variables.kRMaxSpeed) +
        # #     (self.rotLimiter.calculate(
        # #         wpimath.applyDeadband(self.controller.getRawAxis(4), 0.2) # VAR
        # #     )
        # #     * variables.kRMaxSpeed)
        # # )
        # # rot = (
        # #     (-self.rotLimiter.calculate(
        # #         wpimath.applyDeadband(self.controller.getRightY(), 0.2) # VAR
        # #     )
        # #     * variables.kRMaxSpeed) +
        # #     (self.rotLimiter.calculate(
        # #         wpimath.applyDeadband(self.controller.getRightX(), 0.2) # VAR
        # #     )
        # #     * variables.kRMaxSpeed)
        # # )
        # rot = (
        #     -self.rotLimiter.calculate(
        #         wpimath.applyDeadband(self.controller.getRightX(), 0.02)
        #     )
        #     * variables.kMaxSpeed
        # )

        # # variables.setTurnState(rot)

        # #self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
        # self.logDrive(xSpeed=xSpeed,ySpeed=ySpeed,rot=rot)
        # self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

        # if self.controller.getLeftBumperPressed():
        #     self.liftDirection = True if self.liftDirection == False else False

        # if self.controller.getLeftBumper():
        #     self.shooterDirection = True if self.shooterDirection == False else False

        # intakeSpeed = (
        #     self.controller.getRightTriggerAxis() * 0.75 if self.liftDirection == False else
        #     self.controller.getRightTriggerAxis() * -0.75 if self.liftDirection == True else
        #     0
        # )

        # shooterSpeed = (
        #     self.controller.getLeftTriggerAxis() * 0.75 if self.controller.getLeftTriggerAxis and self.shooterDirection == False else
        #     self.controller.getLeftTriggerAxis() * -0.75 if self.controller.getLeftTriggerAxis and self.shooterDirection == True else
        #     0
        # )

        liftSpeed = (wpimath.applyDeadband(self.controller.getRightY(), 0.02))

        self.arm.lift1.set(liftSpeed*0.1)
        self.arm.lift2.set(-liftSpeed*0.1)
        self.arm.intake.set(intakeSpeed)
        self.arm.shooterTop.set(shooterSpeed)
        self.arm.shooterBottom.set(shooterSpeed)

        
