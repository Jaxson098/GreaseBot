#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.kinematics
import wpimath.controller
import variables

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
        self.shootMotor1ID = WPI_TalonSRX(14) # VAR
        self.shootMotor2ID = WPI_TalonSRX(15) # VAR
        
        # NOTE: can we use the wpilib.encoder library for these encoders - may need to review

        # NOTE: Need to determine if this is the right distance per pulse value (from user guide it shows 42 counts per revolution)


   
    def ampshootmotor(self):
        self.shootMotor1ID.set(1) # VAR
        self.shootMotor2ID.set(2) # VAR

    
    def speakershootmotor(self):
        self.shootMotor1ID.set(5) # VAR
        self.shootMotor2ID.set(5) # VAR

    def stopmotor(self):
        self.shootMotor1ID.set(0) # VAR
        self.shootMotor2ID.set(0) # VAR

