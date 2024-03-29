#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
from rev import CANEncoder
from rev import CANSparkMax
from phoenix6 import hardware

kWheelRadius = 0.0508 # In meters
kEncoderResolution = 4096
kVEncoderResolution = 42
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau

class SwerveModule:
    def __init__(
        self,
        driveMotorID: int,
        turningMotorID: int,
        #if driveEncoderID is never used anywhere and is just a parameter cant we just get rid of it?
        # driveEncoderID: int,
        turningEncoderID: int,
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        BERT NOTES:

        - Using different encoder types, current module assumes MXP quadrature encoder that takes 2 channels.  
        - Will need to modify to enable encoders based on CAN ID.
        
        Front Left (+,+) 
            - Drive Motor: 4
            - Rotation Motor: 3
            - Drive Encoder: 13
            - Rotation Encoder: 3 
        
        Front Right (+,-) 
            - Drive Motor: 7
            - Rotation Motor: 8
            - Drive Encoder: 10
            - Rotation Encoder: 8

        Rear Left (-,+) 
            - Drive Motor: 2
            - Rotation Motor: 1
            - Drive Encoder: 11
            - Rotation Encoder: 1

        Rear Right (-,-) 
            - Drive Motor: 5
            - Rotation Motor: 6
            - Drive Encoder: 12
            - Rotation Encoder: 6
            
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

        # Encoders
        self.frontLeftModule_encoder = CANCoder(13, "rio")
        self.frontRightModule_encoder = CANCoder(10, "rio")
        self.rearLeftModule_encoder = CANCoder(11, "rio")
        self.rearRightModule_encoder = CANCoder(12, "rio")

        # Rotation Encoders - check if we need absolute or relative encoder
        self.frontLeftModule_rot_encoder = SparkMaxAbsoluteEncoder(3)
        self.frontRightModule_rot_encoder = SparkMaxAbsoluteEncoder(8)
        self.rearLeftModule_rot_encoder = SparkMaxAbsoluteEncoder(1)
        self.rearRightModule_rot_encoder = SparkMaxAbsoluteEncoder(6)

        """

        # NOTE: need to confirm output from SparkMaxAbsoluteEncoder - may shift to Relative Encoder
        ## No longer required 
        self.driveMotor = CANSparkMax(driveMotorID, CANSparkMax.MotorType.kBrushless)
        self.turningMotor = CANSparkMax(turningMotorID, CANSparkMax.MotorType.kBrushless)
        #would you not put driveEncoderID in thre parenthisies affter .getEncoder?
        self.driveEncoder = self.driveMotor.getEncoder()
        # self.turningEncoder = CANEncoder(turningEncoderID, "rio")
        self.turningEncoder = hardware.CANcoder(turningEncoderID)

        print("encoder", turningEncoderID, self.turningEncoder.get_position())
        #self.turningEncoder = CANCoderConfiguration()
        #self.turningEncoder.sensorCoefficent = math.tau / kEncoderResolution
        #self.configs = CANCoderConfiguration()
        #self.configs = self.turningEncoder.getAllConfigs()
        #print(self.configs)
        #self.turningEncoder.setConfigFeedbackCoefficient = math.tau / kEncoderResolution

        # NOTE: can we use the wpilib.encoder library for these encoders - may need to review

        # NOTE: This is the values we need to tweak 99, 102, 114, & 115
        # Gains are for example purposes only - must be determined for your own robot!

        #how do we determine the PID values?
        self.drivePIDController = wpimath.controller.PIDController(0, 0, 0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            0,
            0,
            0,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )

        # Gains are for example purposes only - must be determined for your own robot!
        # NOTE: To review
        # https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html
        # FeedForward converts

        #how do we determine the voltage?
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0, 0)

        # Set the distance per pulse for the drive encoder. We can simply use the
        # distance traveled for one rotation of the wheel divided by the encoder
        # resolution.

        # NOTE: Need to determine if this is the right distance per pulse value (from user guide it shows 42 counts per revolution)

        self.driveEncoder.setVelocityConversionFactor(
            math.tau * kWheelRadius / kVEncoderResolution
        )

        #NOTE: Review this again: They use RADIANS instaed of DEGREES

        # Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        # This is the the angle through an entire rotation (2 * pi) divided by the
        # encoder resolution.

        # NOTE: Need to determine if this is the right distance per pulse value
        #self.turningEncoder.configFeedbackCoefficient(math.tau / kEncoderResolution)


        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    @staticmethod
    def degree_to_rad(degree):
        """
        Convert a given voltage value to rad.

        :param voltage: a voltage value between 0 and 5
        :returns: the radian value betwen 0 and 2pi
        """
        return degree * math.pi / 180

        #this method is never called, can we get rid of it?
    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        # NOTE: Need to determine if getVelocity value aligns with the expected value vs getRate
        degree= self.turningEncoder.get_position().value
        print(degree)
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.degree_to_rad(degree)),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        degree= self.turningEncoder.get_position().value
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getVelocity(), 
            wpimath.geometry.Rotation2d(self.degree_to_rad(degree)),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """
        degree= self.turningEncoder.get_position().value
        encoderRotation = wpimath.geometry.Rotation2d(self.degree_to_rad(degree))

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoderRotation).cos()

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getVelocity(), state.speed
        )


        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        #NOTE: getPosition may be changed to getAbsolutePosition

        # Calculate the turning motor output from the turning PID controller.
        position = self.turningEncoder.get_position()
        turnOutput = self.turningPIDController.calculate(
            self.degree_to_rad(position.value), state.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        #print(self.turningEncoder.getPosition())

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)
