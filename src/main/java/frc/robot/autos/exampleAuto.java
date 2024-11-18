package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(SwerveSubsystem s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    //Use the lower speed of the set speed or the physical limit for the motor speeds to calculate a trajectory
                    Math.min(Constants.Auto.kMaxSpeedMetersPerSecond,SwerveConstants.maxWheelSpeed),
                    Constants.Auto.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(SwerveSubsystem.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);
                
        var thetaController = 
            new ProfiledPIDController(
                Constants.Auto.kPThetaController, 
                0, 0,
                new TrapezoidProfile.Constraints(
                    Constants.Auto.kMaxAngularSpeedRadiansPerSecond, 
                    Constants.Auto.kMaxAngularAccelerationRadiansPerSecondSquared)
            );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        /*TODO: Swerve controller command finishes whenever the predicted trajectory time elapses
         If the robot was slower than predicted, it won't get all the way to the desired position.
         It also does not zero the drive voltages at the end of a path, but it will default back 
         to the drive command, which would stop it.
        */
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                SwerveSubsystem.swerveKinematics,
                new PIDController(Constants.Auto.kPXController, 0, 0),
                new PIDController(Constants.Auto.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}