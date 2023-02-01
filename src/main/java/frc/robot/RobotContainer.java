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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.PPTrajectoryCommand;
import frc.robot.commands.autonomous.AutoPathMerger;
import frc.robot.commands.autonomous.DriveForDistanceCommand;
import frc.robot.commands.autonomous.DriveForTimeCommand;
import frc.robot.commands.autonomous.DriveRamseteTrajectory;
import frc.robot.subsystems.DriveSubsystem;


public class RobotContainer {

    public static final Joystick stick = new Joystick(0);

    //private final SendableChooser<Command> chooser = new SendableChooser<>();

    // Subsystems
    public static final DriveSubsystem drivetrain = new DriveSubsystem();
    public static AutoPathMerger m_Merger = new AutoPathMerger(drivetrain, AutoConstants.kFirstFileNameUpWest, true, AutoConstants.kParkLowerKeyword);

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
        //return m_Merger.getFollowConcPathWithEvents();
        return m_Merger.getMergedPathCommand();
    }

    public void simulationPeriodic() {
        drivetrain.simulationPeriodic();
    }
}
