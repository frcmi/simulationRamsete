// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.autonomous.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;

import java.io.Console;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public final class Autos {
  // Command used for auto is set at bottom of RobotContainer.java in getAutonomousCommand()
  
  public static CommandBase spinAuto(DriveSubsystem m_drivetrain) {
    return Commands.repeatingSequence(new TurnToAngle(-90, m_drivetrain), new TurnToAngle(90, m_drivetrain));
  }

  public static CommandBase goForSeconds(Double seconds, DriveSubsystem m_Drivetrain) {
    return Commands.sequence(
      Commands.runOnce(() -> m_Drivetrain.arcadeDrive(1, 0), m_Drivetrain),
      Commands.waitSeconds(seconds), 
      Commands.runOnce(() -> m_Drivetrain.stop(), m_Drivetrain));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  
  public static CommandBase combineAutosPoorly(String auto1, String auto2, DriveSubsystem drivetrain, boolean keepRotation) {
    //making classes for the autobuilder cause organization is good i think idk
    RamseteController ramsController = new RamseteController();
    PIDConstants pidConstants = new PIDConstants(SmartDashboard.getNumber("1Feedforward s", 0), SmartDashboard.getNumber("2Feedforward v", 0), SmartDashboard.getNumber("3Feedforward a", 0));
    SimpleMotorFeedforward motorFeedforward = new SimpleMotorFeedforward(SmartDashboard.getNumber("4Pidconstants p", 0), SmartDashboard.getNumber("5Pidconstants i", 0), SmartDashboard.getNumber("6Pidconstants d", 0));

    Map<String, Command> eventHashMap = new HashMap<>();

    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
        drivetrain::getPose, drivetrain::resetOdometry, ramsController, drivetrain.diffDriveKine, motorFeedforward,
         drivetrain::getWheelSpeeds, pidConstants, drivetrain::tankDriveVolts, eventHashMap, drivetrain);
    PathPlannerTrajectory path1 = PathPlanner.loadPath(auto1, new PathConstraints(4, 3));
    PathPlannerTrajectory path2 = PathPlanner.loadPath(auto2, new PathConstraints(4, 3));

    ArrayList<PathPlannerTrajectory> pathFinal = new ArrayList<>();

    Pose2d pose2dOfFirst = path2.getState(0).poseMeters;
    Pose2d pose2dOfLast = path1.getState(path1.getStates().size() - 1).poseMeters;

    pathFinal.add(path1);

    for (int i = 0; i < path2.getStates().size(); i++) {
      Pose2d pose2dOfAdd = path2.getState(i).poseMeters;
      Pose2d pose2dFinal;
      if (keepRotation) {
        pose2dFinal = new Pose2d(
          pose2dOfAdd.getX() + pose2dOfLast.getX() - pose2dOfFirst.getX(), 
          pose2dOfAdd.getY() + pose2dOfLast.getY() - pose2dOfFirst.getY(), 
          pose2dOfAdd.getRotation());
      } else {
        pose2dFinal = new Pose2d(
          pose2dOfAdd.getX() + pose2dOfLast.getX() - pose2dOfFirst.getX(), 
          pose2dOfAdd.getY() + pose2dOfLast.getY() - pose2dOfFirst.getY(), 
          new Rotation2d(pose2dOfAdd.getRotation().getDegrees() - pose2dOfLast.getRotation().getDegrees()));
      }
      path2.getState(i).poseMeters = pose2dFinal;
    }

    pathFinal.add(path2);
    /*
    RamseteCommand ramseteCommand = new RamseteCommand(
      finalTraj, 
      drivetrain::getPose, 
      ramsController, 
      motorFeedforward, 
      DriveConstants.kDriveKinematics, 
      drivetrain::getWheelSpeeds, 
      pidConstants, 
      pidConstants,
      drivetrain::tankDriveVolts, 
      drivetrain);
    */


    return autoBuilder.fullAuto(pathFinal);
  }
}