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
import shooter
from wpimath.kinematics import SwerveModuleState
# import navxGyro
import wpiutil
from wpiutil import wpistruct
import dataclasses
from wpilib import Timer
@wpiutil.wpistruct.make_wpistruct(name="CANCoders")
@dataclasses.dataclass
class CANCodersPosition:
    frontLeftTurn:wpistruct.double = 0.0
    frontRightTurn:wpistruct.double = 0.0
    backLeftTurn:wpistruct.double = 0.0
    backRightTurn:wpistruct.double = 0.0


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        # self.controller = wpilib.Joystick(2) # VAR
        self.controller = wpilib.XboxController(0) # VAR
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


        # Align the wheels to 0
        self.swerve.alignment()

    def logSwerveStates(self):
        frontLeftState = self.swerve.frontLeft.getState()
        frontRightState = self.swerve.frontRight.getState()
        backLeftState = self.swerve.backLeft.getState()
        backRightState = self.swerve.backRight.getState()
        self.pubSS.set([frontLeftState,frontRightState,backLeftState,backRightState])

    def logCANEncoders(self):
        canPost = CANCodersPosition()
        canPost.frontLeftTurn = self.swerve.frontLeft.turningEncoder.getPosition()
        canPost.frontRightTurn = self.swerve.frontLeft.turningEncoder.getPosition()
        canPost.backLeftTurn = self.swerve.frontLeft.turningEncoder.getPosition()
        canPost.backRightTurn = self.swerve.frontLeft.turningEncoder.getPosition()
        self.pubCE.set(canPost)

    #FUTURE
    def autonomousPeriodic(self) -> None:
        #self.driveWithJoystick(False)
        self.swerve.updateOdometry()

    def teleopPeriodic(self) -> None:
        if self.timer.hasElapsed(1):
            self.logSwerveStates()
            # self.logCANEncoders()
            self.timer.restart()
        
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
        xSpeed = (
            -self.xspeedLimiter.calculate(
                # wpimath.applyDeadband(self.controller.getRawAxis(1), 0.1) # VAR
                wpimath.applyDeadband(self.controller.getLeftY(), 0.02) # VAR
            )
            * variables.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        # NOTE: Check if we need inversion here
        ySpeed = (
            -self.yspeedLimiter.calculate(
                # wpimath.applyDeadband(self.controller.getRawAxis(2), 0.6) # VAR
                wpimath.applyDeadband(self.controller.getLeftX(), 0.02) # VAR
            )
            * variables.kTMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        # rot = (
        #     (-self.rotLimiter.calculate(
        #         wpimath.applyDeadband(self.controller.getRawAxis(3), 0.2) # VAR
        #     )
        #     * variables.kRMaxSpeed) +
        #     (self.rotLimiter.calculate(
        #         wpimath.applyDeadband(self.controller.getRawAxis(4), 0.2) # VAR
        #     )
        #     * variables.kRMaxSpeed)
        # )
        # rot = (
        #     (-self.rotLimiter.calculate(
        #         wpimath.applyDeadband(self.controller.getRightY(), 0.2) # VAR
        #     )
        #     * variables.kRMaxSpeed) +
        #     (self.rotLimiter.calculate(
        #         wpimath.applyDeadband(self.controller.getRightX(), 0.2) # VAR
        #     )
        #     * variables.kRMaxSpeed)
        # )
        rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRightX(), 0.02)
            )
            * variables.kMaxSpeed
        )

        # variables.setTurnState(rot)

        #self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

        
