def __init__(self, degrees):
        self.degrees = degrees % 360  # Ensure angle is within 0-359 degrees

    def set_angle(self, degrees):
        self.degrees = degrees % 360

    def get_angle(self):
        return self.degrees

    def rotate(self, delta_degrees):
        self.degrees = (self.degrees + delta_degrees) % 360

# Example usage:
angle = ContinuousAngle(361)  # Creating an angle of 361 degrees
print(angle.get_angle())      # Output: 1 (since 361 % 360 = 1)

angle.rotate(180)             # Rotating angle by 180 degrees
print(angle.get_angle())      # Output: 181

angle.rotate(-360)            # Rotating angle by -360 degrees
print(angle.get_angle())      # Output: 181 (no change because -360 % 360 = 0)

