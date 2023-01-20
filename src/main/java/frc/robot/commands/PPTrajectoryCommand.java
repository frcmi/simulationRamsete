package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;


public class PPTrajectoryCommand extends CommandBase {

    protected DriveSubsystem mDriveSubsystem;
    protected String nameOfFile;
    protected PathPlannerTrajectory traj;

    public PPTrajectoryCommand(DriveSubsystem driveSubsystem, String fileName) {
        mDriveSubsystem = driveSubsystem;
        nameOfFile = fileName;
        PathPlannerTrajectory ppTraj = PathPlanner.loadPath(nameOfFile, new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        traj = ppTraj;
    }

    public Command followTrajectoryCommand(boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
              // Reset odometry for the first path you run during auto
              if(isFirstPath){
                  mDriveSubsystem.resetOdometry(traj.getInitialPose());
              }
            }),
            new PPRamseteCommand(
                traj, 
                mDriveSubsystem::getPose, // Pose supplier
                new RamseteController(),
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, // DifferentialDriveKinematics
                mDriveSubsystem::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                new PIDController(AutoConstants.kTurnP, AutoConstants.kTurnI, AutoConstants.kTurnD), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(AutoConstants.kTurnP, AutoConstants.kTurnI, AutoConstants.kTurnD), // Right controller (usually the same values as left controller)
                mDriveSubsystem::tankDriveVolts, // Voltage biconsumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                mDriveSubsystem // Requires this drive subsystem
            )
        );
    }

}