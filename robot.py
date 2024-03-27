#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
# main code/ iniations
#
#
#

import ntcore
import wpilib
import wpimath
from wpilib.cameraserver import CameraServer
import wpilib.drive
import wpimath.filter
import wpimath.controller
from wpimath.kinematics import SwerveModuleState
from components import drivetrain
from components import arm
from rev import CANSparkMax



class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        #CONROLLERS 
        self.controller = wpilib.XboxController(0)
        self.swerve = drivetrain.Drivetrain()
        self.arm = arm.Arm()

        # get the default instance of NetworkTables
        nt = ntcore.NetworkTableInstance.getDefault()

        # Start publishing an array of module states with the "/SwerveStates" key
        topic = nt.getStructArrayTopic("/SwerveStates", SwerveModuleState)
        self.pub = topic.publish()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        # Launch Camera
        #wpilib.CameraServer.launch()

    def autonomousPeriodic(self) -> None:
        self.driveWithJoystick(False)
        self.swerve.updateOdometry()

    def teleopPeriodic(self) -> None:
        #self.pub.set([frontLeftState,frontRightState,backLeftState,backRightState])
        self.driveWithJoystick(True)

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        # NOTE: Check if we need inversion here
        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftY(), 0.02)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        # NOTE: Check if we need inversion here
        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftX(), 0.02)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRightX(), 0.02)
            )
            * drivetrain.kMaxSpeed
        )

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

        IntakeDirection = True
        if self.controller.getRightBumper():
            IntakeDirection = False

        ShooterDirection = True
        if self.controller.getLeftBumper():
            ShooterDirection = False

        intakeSpeed = (
            self.controller.getRightTriggerAxis() * 0.75 if self.controller.getRightTriggerAxis and IntakeDirection == True else
            self.controller.getRightTriggerAxis() * -0.75 if self.controller.getRightTriggerAxis and IntakeDirection == False else
            0
        )

        shooterSpeed = (
            self.controller.getLeftTriggerAxis() * 0.75 if self.controller.getLeftTriggerAxis and ShooterDirection == True else
            self.controller.getLeftTriggerAxis() * -0.75 if self.controller.getLeftTriggerAxis and ShooterDirection == False else
            0
        )

        liftSpeed = (wpimath.applyDeadband(self.controller.getRightY(), 0.02)) * 0.75

        self.arm.lift1.set(liftSpeed)
        self.arm.lift2.set(liftSpeed)
        self.arm.intake.set(intakeSpeed)
        self.arm.shooterTop.set(shooterSpeed)
        self.arm.shooterBottom.set(shooterSpeed)