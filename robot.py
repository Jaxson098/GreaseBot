#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import drivetrain
import variables
import shooter
import navxGyro


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.Joystick(2)
        #self.controller = wpilib.PS4Controller(2)
        #self.controller = wpilib.XboxController(0)
        self.swerve = drivetrain.Drivetrain()
        self.shooter = shooter.ShootModule()
        # navxGyro is a file to test the navx Gyro. This can be ignored/commented out.
        self.navxGyro = navxGyro.Gyro()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        # Speed limiters

        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(4)

        # Align the wheels to 0
        #self.swerve.alignment()

    #FUTURE
    def autonomousPeriodic(self) -> None:
        #self.driveWithJoystick(False)
        self.swerve.updateOdometry()

    def teleopPeriodic(self) -> None:
        self.driveWithJoystick(False)
        self.navxGyro.getGyro()
        #self.shootWithJoystick(False)
        #self.shooter.speakershootmotor(1, 1)
        if self.controller.getRawButton(4) == 1:
            self.swerve.drive(0,0,0,0,self.getPeriod())
            self.swerve.alignment()

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        # NOTE: Check if we need inversion here
        xSpeed = (
            self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(1), 0.1)
            )
            * variables.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        # NOTE: Check if we need inversion here
        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(2), 0.6)
            )
            * variables.kTMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            (-self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(3), 0.5)
            )
            * variables.kRMaxSpeed) +
            (self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(4), 0.5)
            )
            * variables.kRMaxSpeed)
        )

        variables.setTurnState(rot)

        #self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

        
