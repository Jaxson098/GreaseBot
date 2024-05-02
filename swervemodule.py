import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
from phoenix5.sensors import CANCoder
import phoenix5.sensors 
from rev import CANSparkMax
import wpimath.units
import drivetrain
import variables


kWheelRadius = 0.0508 #NOTE: do we need to change this?
kEncoderResolution = 4096 #NOTE: is this correct?
kVEncoderResolution = 42 #NOTE: can we get rid of this since we dont use drive encoder
kModuleMaxAngularVelocity = math.pi 
kModuleMaxAngularAcceleration = math.tau

class SwerveModule:
    def __init__(
        self,
        driveMotorID: int,
        turningMotorID: int,
        driveEncoderID: int,
        turningEncoderID: int,
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder."""

        #1 swerve module has a drive motor, turning motor, and turn encoder (cancoder)
        self.driveMotor = CANSparkMax(driveMotorID, CANSparkMax.MotorType.kBrushless)
        self.turningMotor = CANSparkMax(turningMotorID, CANSparkMax.MotorType.kBrushless)
        #NOTE: can we get rid of this
        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = CANCoder(turningEncoderID, "rio")

        #config measurments
        self.turningEncoder.configFeedbackCoefficient((2 * math.pi / 4096), "rad", phoenix5.sensors.SensorTimeBase(1))
        #NOTE: this overrides boot to zero setting on the encoders so thats why we had the align problem
        self.turningEncoder.configAbsoluteSensorRange(phoenix5.sensors.AbsoluteSensorRange(1))
        self.turningEncoder.configSensorDirection(1)

        #initilizes a PID controler for drive motors useing P I and D values in variables.py
        self.drivePIDController = wpimath.controller.PIDController(variables.drivePID_P, variables.drivePID_I, variables.drivePID_D)

        #initilizes a PID controler for turn motors useing P I  D values max speed and acelaration in variables.py 
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            variables.turnPID_P,
            variables.turnPID_I,
            variables.turnPID_D,
            #makes a TrapezoidProfile useing the max speed and acelaaration (speed graph looks like a trapazoid)
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )

        #initializes feedforward controlers for turn and drive motors which predict the amount of voltadge needed based of of FF 1 and FF 2 (see variables)
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(variables.driveFF_1, variables.driveFF_2)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(variables.turnFF_1, variables.turnFF_2)

        #NOTE: can we get rid of this
        #takes the radius in meeters x 2PI (the circumfrence) dived by the resolution, returning the pulse per 1 meeter traveles
        self.driveEncoder.setVelocityConversionFactor(
            math.tau * kWheelRadius / kVEncoderResolution
        )

        #input range is bewtween -PI and PI (1 half of a circle or the other half)
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        #used for logging purposes
        """Returns the current state of the module"""

        return wpimath.kinematics.SwerveModuleState(
            #NOTE: can we get rid of this
            #get the velocity of the Drive encoder
            self.driveEncoder.getVelocity(),
            #get the position of the turningEncoder as a rotaiton 2d object
            wpimath.geometry.Rotation2d(self.turningEncoder.getPosition()),
        )
    
    #NOTE: what is the diffrence bewtween these 2 methods

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        #used for odomotry
        """Returns the current position of the module."""

        return wpimath.kinematics.SwerveModulePosition(
            #NOTE: can we get rid of this
            #get the velocity of the Drive encoder
            self.driveEncoder.getVelocity(),
            #get the position of the turningEncoder as a rotaiton 2d object
            wpimath.geometry.Rotation2d(self.turningEncoder.getPosition()),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module."""
        
        #variable that gets the position of the can encoder
        encoderRotation = wpimath.geometry.Rotation2d(self.turningEncoder.getPosition())

        #optimize the wheel turn so that it only has to turn 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        #take the optimized direction and find the distance needed to travel by solving for θ
        #useing cosine to solve for θ since hypotanouse is encoder position and adjacent is desired state of that wheel
        state.speed *= (state.angle - encoderRotation).cos()

        # Calculate the output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getVelocity(), state.speed
        )

        #calculates the amount of voltage needed
        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.getPosition(), state.angle.radians() 
        )

        #caluclates the amojunt fo voltage needed
        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        #setting the power for the turn and drive motor based off of the pid controler and drive feed forward calculations
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)
        self.driveMotor.setVoltage(driveOutput + driveFeedforward)