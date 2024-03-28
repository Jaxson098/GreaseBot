from rev import CANSparkMax

class Arm:
    def motor(self, motorID: int, encoderID: int,):
        return CANSparkMax(motorID, encoderID)
    
    def __init__(self):
        self.lift1 = self.motor(14, 14)
        self.lift2 = self.motor(14, 14)
        self.intake = self.motor(14, 14)
        self.shooterTop = self.motor(14, 14)
        self.shooterBottom = self.motor(14, 14)