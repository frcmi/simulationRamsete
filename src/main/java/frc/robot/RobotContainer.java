package frc.robot;

import java.lang.ModuleLayer.Controller;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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
        SmartDashboard.setDefaultNumber("left speed", 1);
        /*
        s
        v
        a
        p
        i
        d
        */
        /*chooser.setDefaultOption("Drive Ramsete Trajectory", 
                new DriveRamseteTrajectory(drivetrain));
        chooser.addOption("Drive For Distance",
                new DriveForDistanceCommand(drivetrain, 0.75, 5.0));
        chooser.addOption("Drive For Time",
                new DriveForTimeCommand(drivetrain, 0.75, 3.0));
        SmartDashboard.putData("Autonomous Options", chooser);
        */
        
    }

    PIDConstants pidConstants = new PIDConstants(1, 0, 0);
    SimpleMotorFeedforward motorFeedforward = new SimpleMotorFeedforward(1, 0, 1);

    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
        drivetrain::getPose, drivetrain::resetOdometry, ramsController, drivetrain.diffDriveKine, motorFeedforward,
         drivetrain::getWheelSpeeds, pidConstants, drivetrain::tankDriveVolts, eventHashMap, drivetrain);

    ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("PickupAndDeliver", new PathConstraints(4, 3));
    Command fullAuto  = autoBuilder.fullAuto(pathGroup);

    public Command getAutonomousCommand() {
        return fullAuto;
    }

    public void simulationPeriodic() {
        drivetrain.simulationPeriodic();
    }
}
