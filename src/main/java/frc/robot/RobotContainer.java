package frc.robot;

import java.util.List;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.autonomous.DriveForDistanceCommand;
import frc.robot.commands.autonomous.DriveForTimeCommand;
import frc.robot.commands.autonomous.DriveRamseteTrajectory;
import frc.robot.subsystems.DrivetrainTalonFXSubsystem;


public class RobotContainer {

    public static final Joystick stick = new Joystick(0);

    //private final SendableChooser<Command> chooser = new SendableChooser<>();

    // Subsystems
    public static final DrivetrainTalonFXSubsystem drivetrain = new DrivetrainTalonFXSubsystem();

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        drivetrain.setDefaultCommand(new JoystickDriveCommand(drivetrain,
                () -> -stick.getY(),
                stick::getX,
                () -> ((stick.getZ() - 1) / -2.0)
                ));

        configureButtonBindings();

        initializeAutonomousOptions();
    }

    private void configureButtonBindings() {

    }

    private void initializeAutonomousOptions() {
        /*chooser.setDefaultOption("Drive Ramsete Trajectory", 
                new DriveRamseteTrajectory(drivetrain));
        chooser.addOption("Drive For Distance",
                new DriveForDistanceCommand(drivetrain, 0.75, 5.0));
        chooser.addOption("Drive For Time",
                new DriveForTimeCommand(drivetrain, 0.75, 3.0));
        SmartDashboard.putData("Autonomous Options", chooser);
        */
        
    }

    public Command getAutonomousCommand() {
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
                drivetrain::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                drivetrain::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drivetrain::tankDriveVolts,
                drivetrain);

        // Reset odometry to the starting pose of the trajectory.
        drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    }

    public void simulationPeriodic() {
        drivetrain.simulationPeriodic();
    }
}
