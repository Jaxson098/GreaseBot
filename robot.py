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
from rev import CANSparkMax
from config import FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_TURNING_MOTOR_ID
from wpilib import DataLogManager, DriverStation
from wpilib import Timer
from wpiutil.log import (
    DataLog,
    BooleanLogEntry,
    DoubleLogEntry,
    StringLogEntry,
)


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""

        DataLogManager.start()
        # Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog())
        log: DataLog = DataLogManager.getLog()
        
        self.timer = Timer()
        self.timer.start()
        self.ticks = 0

        # CONROLLERS
        self.liftDirection = False
        self.shooterDirection = False

        # self.controller = wpilib.XboxController(0)
        self.controller = wpilib.PS4Controller(0)
        self.swerve = drivetrain.Drivetrain()

        # get the default instance of NetworkTables
        nt = ntcore.NetworkTableInstance.getDefault()
        # Start publishing an array of module states with the "/SwerveStates" key
        # from https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
        # topic = nt.getStructArrayTopic("/SwerveStates", SwerveModuleState)
        topic = nt.getStringTopic("/adrian")
        self.pub = topic.publish()

        topicSS = nt.getStructArrayTopic("/SwerveStatesAdrian", SwerveModuleState)
        self.pubSS = topicSS.publish()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        # Launch Camera
        # wpilib.CameraServer.launch()

    def autonomousPeriodic(self) -> None:
        self.driveWithJoystick(False)
        self.swerve.updateOdometry()

    def onTimer(self):
            self.ticks += 1
            self.pub.set(f"the timer is {self.ticks}")

            frontLeftState = self.swerve.frontLeft.getState()
            frontRightState = self.swerve.frontRight.getState()
            backLeftState = self.swerve.backLeft.getState()
            backRightState = self.swerve.backRight.getState()

            self.pubSS.set([frontLeftState,frontRightState,backLeftState,backRightState])
            self.timer.reset()


    def teleopPeriodic(self) -> None:
        if self.timer.hasElapsed(0.5):
            self.onTimer()

        # self.pub.set([frontLeftState,frontRightState,backLeftState,backRightState])
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

        print(f"rotation {rot} xspeed{xSpeed}")

        # driving the robot
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
