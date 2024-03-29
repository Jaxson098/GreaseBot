from wpilib.command import Command
from wpilib.trajectory import Trajectory, TrajectoryConfig
from wpilib.trajectory.constraint import DifferentialDriveVoltageConstraint
from wpilib.controller import RamseteController, SimpleMotorFeedforward
from wpilib.geometry import Pose2d, Transform2d
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry


class AutonomousSwerveDrive:

    def __init__(self):
        # Define module locations FIX THIS JACKSON, I KNOW THIS IS DIFFERENT
        frontLeftLocation = Translation2d(0.381, 0.381)
        frontRightLocation = Translation2d(0.381, -0.381)
        backLeftLocation = Translation2d(-0.381, 0.381)
        backRightLocation = Translation2d(-0.381, -0.381)

        # Create kinematics object

        self.kinematics = SwerveDrive4Kinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
        )

        # Initialize odometry object (you'll need to fill in the initial module positions and pose)
        # FIX THIS, I KNOW IT’S NOT A ROTATION2D CARTESIAN

        self.odometry = SwerveDrive4Odometry(
            self.kinematics, Rotation2d(),
            (0, 0, 0, 0), # Placeholder for module positions
            Pose2d(0, 0, Rotation2d()) # Placeholder for initial pose
        )


    def generateTrajectory(self, waypoints):

        # Generate a trajectory based on the waypoints

        # This is a simplified example; you'll need to fill in the details

        trajectoryConfig = TrajectoryConfig(

            maxVelocity=3.0, # m/s

            maxAcceleration=3.0, # m/s^2

            reversed=False,

            startPose=Pose2d(),

            endPose=Pose2d()

        )

        trajectory = Trajectory(waypoints, trajectoryConfig)

        return trajectory


    def followTrajectory(self, trajectory):

        # Implement the command to follow the trajectory

        # This is a simplified example; you'll need to fill in the details

        class FollowTrajectoryCommand(Command):

            def __init__(self, trajectory):

                super().__init__()

                self.trajectory = trajectory

                self.requires(self.robot.driveSubsystem)


            def initialize(self):

                self.controller = RamseteController(2, 0.7)

                self.feedforward = SimpleMotorFeedforward(1, 3, 1)

                self.pose = self.robot.driveSubsystem.getPose()

                self.timer = self.getRobot().getTimer()

                self.timer.start()


            def execute(self):

                currentTime = self.timer.get()

                desiredState = self.trajectory.sample(currentTime)

                poseError = desiredState.pose - self.robot.driveSubsystem.getPose()

                steeringAdjust = self.controller.calculate(self.robot.driveSubsystem.getPose(), desiredState.pose)

                feedforward = self.feedforward.calculate(desiredState.velocity, poseError.curvature)

                output = steeringAdjust + feedforward

                self.robot.driveSubsystem.drive(output, output)


            def isFinished(self):

                return self.timer.get() >= self.trajectory.getTotalTimeSeconds()


            def end(self):

                self.robot.driveSubsystem.stop()


        return FollowTrajectoryCommand(trajectory)


# Example usage

autonomousSwerveDrive = AutonomousSwerveDrive()

trajectory = autonomousSwerveDrive.generateTrajectory(waypoints)

followTrajectoryCommand = autonomousSwerveDrive.followTrajectory(trajectory)

# Schedule the command in your robot's autonomous routine