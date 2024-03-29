#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
from . import swervemodule
import navx
from navx import AHRS


kMaxSpeed = 4.7 
kMaxAngularSpeed = math.pi  # 1/2 rotation per second


class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:

        #adjusted to our IDs
        self.frontLeft = swervemodule.SwerveModule(3, 4, 13)
        self.frontRight = swervemodule.SwerveModule(8, 2, 10)
        self.backLeft = swervemodule.SwerveModule(6, 9, 11)
        self.backRight = swervemodule.SwerveModule(7, 5, 12)

        self.kinimaticsLocation()

        # NOTE: May need to tweak the center measurements 44.5mm rough measurement
        # NOTE: EVERYTHING IS MEASURE IN METERS! 

        #adjusted to our wheel loacations
    def kinimaticsLocation(self):
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.290, 0.290)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.290, -0.290)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.290, 0.290)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.290, -0.290)

        #adjusted to use the navx 
        self.gyro = AHRS.create_spi()

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        #print("kinematics", self.frontLeftLocation, self.frontRightLocation, self.backLeftLocation, self.backRightLocation)
        #print("odometry", self.frontLeft.getPosition(), self.frontRight.getPosition(), self.backLeft.getPosition(), self.backRight.getPosition())

        self.gyro.reset()

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """


        chasisSpeeds=wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)
        translation = wpimath.geometry.Translation2d(0, 0)
        # swerveModuleStates = self.kinematics.toSwerveModuleStates(chasisSpeeds,periodSeconds)
        swerveModuleStates = self.kinematics.toSwerveModuleStates(chasisSpeeds,translation)

        our_variable = "relative field" if fieldRelative else "not relative field"
        # swerveModuleStates = self.kinematics.toSwerveModuleStates(
        #     wpimath.kinematics.ChassisSpeeds.discretize(
        #         wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
        #             xSpeed, ySpeed, rot, self.gyro.getRotation2d()
        #         ))
        #         if fieldRelative
        #         else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot),
        #         periodSeconds,
            
        # )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
        #not sure what the numbers in [] do/are for
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )