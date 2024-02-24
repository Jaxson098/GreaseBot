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




class Intakemodule:
  def__init__(
    self,
    IntakeMoter1ID: int,
    IntakeMotor2ID: int, 
  )-> None:

  self.IntakeMotor1ID = WPI_TalonSRX(16) # VAR
self.IntakeMotor2ID:= WPI_TalonSRX(17) # VAR



def vaccummotor(self):
  self.IntakeMotor1ID.set(-2) # VAR
  self.IntakeMotor2ID.set(-3) # VAR

def stopmotor(self): 
  self.IntakeMotor1ID.set(0) # VAR
  self.IntakeMotor2ID.set(0) # VAR
  

