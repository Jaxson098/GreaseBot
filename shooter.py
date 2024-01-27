#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.kinematics
import wpimath.controller
from phoenix5 import WPI_TalonSRX

# Variables



class ShootModule:
    def __init__(
        self,
        shootMotor1ID: int,
        shootMotor2ID: int,
    ) -> None:

        # NOTE: need to confirm output from SparkMaxAbsoluteEncoder - may shift to Relative Encoder
        ## No longer required 
        self.shootMotor1ID = 
        self.shootMotor2ID = 

        # NOTE: can we use the wpilib.encoder library for these encoders - may need to review

        # NOTE: Need to determine if this is the right distance per pulse value (from user guide it shows 42 counts per revolution)
        self.driveEncoder.setVelocityConversionFactor(
            math.tau * kWheelRadius / kVEncoderResolution
        )

        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        # NOTE: Need to determine if getVelocity value aligns with the expected value vs getRate
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getAbsolutePosition()),
        )
