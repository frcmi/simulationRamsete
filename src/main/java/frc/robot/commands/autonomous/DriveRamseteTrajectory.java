package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainTalonFXSubsystem;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import edu.wpi.first.wpilibj2.command.RamseteCommand;



public class DriveRamseteTrajectory extends CommandBase {
    private final DrivetrainTalonFXSubsystem drivetrainTalonFXSubsystem;

    public DriveRamseteTrajectory(DrivetrainTalonFXSubsystem drivetrainTalonFXSubsystem) {
        this.drivetrainTalonFXSubsystem = drivetrainTalonFXSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.drivetrainTalonFXSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("INFO: DriveForTimeCommand \"initialize\"");
        super.initialize();
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                11);

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, -1), new Translation2d(2, 1)), //new Translation2d(2, 1), new Translation2d(1, -1)
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);

                RamseteCommand ramseteCommand =
                new RamseteCommand(
                    exampleTrajectory,
                    drivetrainTalonFXSubsystem::getPose,
                    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                    new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                    DriveConstants.kDriveKinematics,
                    drivetrainTalonFXSubsystem::getWheelSpeeds,
                    new PIDController(DriveConstants.kPDriveVel, 0, 0),
                    new PIDController(DriveConstants.kPDriveVel, 0, 0),
                    // RamseteCommand passes volts to the callback
                    drivetrainTalonFXSubsystem::tankDriveVolts,
                    drivetrainTalonFXSubsystem);

        // Reset odometry to the starting pose of the trajectory.
        drivetrainTalonFXSubsystem.resetOdometry(exampleTrajectory.getInitialPose());
        ramseteCommand.andThen(() -> drivetrainTalonFXSubsystem.tankDriveVolts(0, 0));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("INFO: DriveForTimeCommand \"end\": interrupted = %b%n", interrupted);
        super.end(interrupted);
        drivetrainTalonFXSubsystem.stop();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
