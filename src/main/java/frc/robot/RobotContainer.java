package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.autonomous.DriveForDistanceCommand;
import frc.robot.commands.autonomous.DriveForTimeCommand;
import frc.robot.commands.autonomous.DriveRamseteTrajectory;
import frc.robot.subsystems.DrivetrainTalonFXSubsystem;


public class RobotContainer {

    public static final Joystick stick = new Joystick(0);

    private final SendableChooser<Command> chooser = new SendableChooser<>();

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
        chooser.setDefaultOption("Drive Ramsete Trajectory", 
                new DriveRamseteTrajectory(drivetrain));
        chooser.addOption("Drive For Distance",
                new DriveForDistanceCommand(drivetrain, 0.75, 5.0));
        chooser.addOption("Drive For Time",
                new DriveForTimeCommand(drivetrain, 0.75, 3.0));
        SmartDashboard.putData("Autonomous Options", chooser);
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

    public void simulationPeriodic() {
        drivetrain.simulationPeriodic();
    }
}
