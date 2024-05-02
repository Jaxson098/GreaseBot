import math
import navx
import wpilib
import wpimath.geometry
import wpimath.kinematics
import swervemodule
import variables

class Drivetrain:

    def __init__(self) -> None:
        
        #kinimatics locations to make a 2d object
        self.frontRightLocation = wpimath.geometry.Translation2d(0.32, 0.32)
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.32, -0.32)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.32, 0.32)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.32, -0.32)

        #IDs
        self.frontLeft = swervemodule.SwerveModule(3, 4, 3, 13)
        self.frontRight = swervemodule.SwerveModule(8, 2, 8, 10)
        self.backLeft = swervemodule.SwerveModule(9, 6, 9, 11)
        self.backRight = swervemodule.SwerveModule(7, 5, 7, 12)

        #setup gyro
        self.angler = navx.AHRS.create_spi()
        self.gyro = self.angler.getAngle()
        self.gyroradians = wpimath.geometry.Rotation2d.fromDegrees(self.gyro)
        #print heading in degrees
        print("gyro", self.gyro)

        #turning individual locations into 1 object
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        #keeps track of robot position ovoer time
        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics, (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
                #NOTE: need to add:
                self.angler.getRotation2d()
                #starting position
            ),
        )

        self.angler.reset()

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        #function uses that data ^ to drive robot

        #takes the overall desired chasis speeds and performs inverse kinimatics (splits it up into desired motion for the 4 module locations)
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            #creates a overall desired speed of the robot in a single time frame, non continues desired motion so it can be calculated
            wpimath.kinematics.ChassisSpeeds.discretize(
                #converts desired field relitive motion into robot relitive desired motion so the robot can actually perform said motion
                wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    #the desired speeds and gyro heading in radians all of this ^ uses
                    xSpeed, ySpeed, rot, self.angler.getRotation2d()
                ) 
                #only convert to robot relitive if field relitive
                if fieldRelative
                #if it is not field relitive dont convert to robot relitive asume it was robot relitive from the beginning
                else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds,
            )
        )

        #makes sure the inverse kinimatics did not have a wheel go over kmax speed, if so reduce all wheel speeds until they are all < Kmaxspeed
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, variables.kMaxSpeed
        )

        #set the diffrent modules to the desired speeds
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        #gets the states of the swerve modules and updates the SwerveDrive4Odometry object
        self.odometry.update(
            wpimath.geometry.Rotation2d(self.gyroradians),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

    def alignment(self) -> None:
        """aligns the wheels"""
        print("aligning")
        #aligns the wheels forward useing the desired angle and the desired speed
        #(speed is nuanced since it uses PID and other optimization calculations in setDesiredState)
        self.frontLeft.setDesiredState(wpimath.kinematics.SwerveModuleState(0.1, wpimath.geometry.Rotation2d.fromDegrees(-177)))
        self.frontRight.setDesiredState(wpimath.kinematics.SwerveModuleState(0.1, wpimath.geometry.Rotation2d.fromDegrees(176)))
        self.backLeft.setDesiredState(wpimath.kinematics.SwerveModuleState(0.1, wpimath.geometry.Rotation2d.fromDegrees(-176)))
        self.backRight.setDesiredState(wpimath.kinematics.SwerveModuleState(0.1, wpimath.geometry.Rotation2d.fromDegrees(10)))