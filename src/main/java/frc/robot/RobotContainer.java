package frc.robot;

import java.lang.ModuleLayer.Controller;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.swing.plaf.basic.BasicTreeUI.TreeIncrementAction;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.TrajectoryFunctions;
import frc.robot.commands.autonomous.DriveForDistanceCommand;
import frc.robot.commands.autonomous.DriveForTimeCommand;
import frc.robot.commands.autonomous.DriveRamseteTrajectory;
import frc.robot.subsystems.DriveSubsystem;


public class RobotContainer {

    public static final Joystick stick = new Joystick(0);
    public static final RamseteController ramsController = new RamseteController();

    //private final SendableChooser<Command> chooser = new SendableChooser<>();

    // Subsystems
    public static final DriveSubsystem drivetrain = new DriveSubsystem();

    private HashMap<String, Command> eventHashMap = new HashMap<>();

    public RobotContainer() {
        eventHashMap.put("PickupCone", new PrintCommand("Picked up cone"));

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
        SmartDashboard.setDefaultNumber("1Feedforward s", 1);
        SmartDashboard.setDefaultNumber("2Feedforward v", 0);
        SmartDashboard.setDefaultNumber("3Feedforward a", 0);
        SmartDashboard.setDefaultNumber("4Pidconstants p", 1);
        SmartDashboard.setDefaultNumber("5Pidconstants i", 0);
        SmartDashboard.setDefaultNumber("6Pidconstatns d", 0);
    }



    public Command getAutonomousCommand() {
        PathConstraints pathConstraints = new PathConstraints(4, 3);

        PathPlannerTrajectory combinedTrajectory = TrajectoryFunctions.combineTrajectories(
            PathPlanner.loadPath("MovePt1", pathConstraints),
            PathPlanner.loadPath("MovePt2", pathConstraints),
            PathPlanner.loadPath("MovePt3", pathConstraints));

        return TrajectoryFunctions.followTrajectoryWithEvents(
            combinedTrajectory, 
            drivetrain, 
            new String[] {"event", "event1"},
            new PrintCommand("halfway done with traj 1!"),
            new PrintCommand("halfway done with traj 2!")
        );
    }

    public void simulationPeriodic() {
        drivetrain.simulationPeriodic();
    }
}
