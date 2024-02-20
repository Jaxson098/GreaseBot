self.gyro = AnalogGyro(0)
 

#navx micro for angles
        self.angler = navx.AHRS.create_i2c()

 self.armExtensionMotor = ctre.WPI_TalonSRX(ArmExtensionMotorPort)
        self.angleMotor = ctre.WPI_TalonSRX(AngleMotorPort)

