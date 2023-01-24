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
}