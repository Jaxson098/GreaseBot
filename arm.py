from rev import CANSparkMax

class Arm:
    def motor(self, motorID: int):
        return CANSparkMax(motorID, CANSparkMax.MotorType.kBrushless)
    
    def __init__(self):
        self.lift1 = self.motor(15)
        self.lift2 = self.motor(16)
        self.intake = self.motor(14)
        self.shooterTop = self.motor(16)
        self.shooterBottom = self.motor(15)