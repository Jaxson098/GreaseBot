#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
import swervemodule

kMaxSpeed = 3.0  # 3 meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second


class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:
        # NOTE: May need to tweak the center measurements 44.5mm rough measurement
        # NOTE: EVERYTHING IS MEASURE IN METERS! 

        self.frontLeftLocation = wpimath.geometry.Translation2d(0.445, 0.445)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.445, -0.445)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.445, 0.445)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.445, -0.445)

        self.frontLeft = swervemodule.SwerveModule(4, 3, 4, 13)
        self.frontRight = swervemodule.SwerveModule(7, 8, 7, 10)
        self.backLeft = swervemodule.SwerveModule(2, 1, 2, 11)
        self.backRight = swervemodule.SwerveModule(5, 6, 5, 12)

        '''
        BERT NOTES:
        
        driveMotorID: int,
        turningMotorID: int,
        driveEncoderID: int,
        turningEncoderID: int,

        Front Left (+,+) 
            - Drive Motor: 4
            - Rotation Motor: 3
            - Drive Encoder: 4
            - Rotation Encoder: 13 
        
        Front Right (+,-) 
            - Drive Motor: 7
            - Rotation Motor: 8
            - Drive Encoder: 7
            - Rotation Encoder: 10

        Rear Left (-,+) 
            - Drive Motor: 2
            - Rotation Motor: 1
            - Drive Encoder: 2
            - Rotation Encoder: 11

        Rear Right (-,-) 
            - Drive Motor: 5
            - Rotation Motor: 6
            - Drive Encoder: 5
            - Rotation Encoder: 12
            
        # Drive Motors
        self.frontLeftModule_driveMotor = CANSparkMax(4, CANSparkMax.MotorType.kBrushless)
        self.frontRightModule_driveMotor = CANSparkMax(7, CANSparkMax.MotorType.kBrushless)
        self.rearLeftModule_driveMotor = CANSparkMax(2, CANSparkMax.MotorType.kBrushless)
        self.rearRightModule_driveMotor = CANSparkMax(5, CANSparkMax.MotorType.kBrushless)
        
        # Rotate Motors
        self.frontLeftModule_rotateMotor = CANSparkMax(3, CANSparkMax.MotorType.kBrushless)
        self.frontRightModule_rotateMotor = CANSparkMax(8, CANSparkMax.MotorType.kBrushless)
        self.rearLeftModule_rotateMotor = CANSparkMax(1, CANSparkMax.MotorType.kBrushless)
        self.rearRightModule_rotateMotor = CANSparkMax(6, CANSparkMax.MotorType.kBrushless)

        # Rotation Encoders
        self.frontLeftModule_encoder = CANCoder(13, "rio")
        self.frontRightModule_encoder = CANCoder(10, "rio")
        self.rearLeftModule_encoder = CANCoder(11, "rio")
        self.rearRightModule_encoder = CANCoder(12, "rio")

        # Drive Encoders - check if we need absolute or relative encoder
        self.frontLeftModule_drive_encoder = SparkMaxAbsoluteEncoder(4)
        self.frontRightModule_drive_encoder = SparkMaxAbsoluteEncoder(7)
        self.rearLeftModule_drive_encoder = SparkMaxAbsoluteEncoder(2)
        self.rearRightModule_drive_encoder = SparkMaxAbsoluteEncoder(5)

        '''

        self.gyro = wpilib.AnalogGyro(0)

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

        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, self.gyro.getRotation2d()
                )
                if fieldRelative
                else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
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
