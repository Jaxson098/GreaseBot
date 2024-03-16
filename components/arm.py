from rev import CANSparkMax

class motor:
    def __init__(self, motorID: int, encoderID: int,) -> None:
        self.motor = CANSparkMax(motorID, encoderID)

    def setPower(self, power: float,):
        self.motor.set(power)

class Arm:
    def __init__(self):
        self.lift1 = motor(14, 14)
        self.lift2 = motor(14, 14)
        self.intake = motor(14, 14)
        self.outtake1 = motor(14, 14)
        self.outtake2 = motor(14, 14)