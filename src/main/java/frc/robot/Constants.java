// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static double rotationMultiplier = 0.5;
    public static double speedMultiplier = 0.5;

    //change ports, measurements
    public static final int kPigeonPort = 1;

    public static final int kLeftMotor1Port = 5;
    public static final int kLeftMotor2Port = 4;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 3;

    public static final double kTrackwidthMeters = 0.62898; //Need to measure
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kWheelCircumference = Math.PI * kWheelDiameterMeters;
    public static final double kGearRatio = 10.9091; 
    public static final double kEncoderDistancePerTick = kWheelCircumference / kGearRatio / kEncoderCPR ;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.91351;
    public static final double kvVoltSecondsPerMeter = 2.5447;
    public static final double kaVoltSecondsSquaredPerMeter = 0.47201;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 3.7728;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kTurnP = 1.0;
    public static final double kTurnI = 1.0;
    public static final double kTurnD = 1.0;
    public static final double kTurnToleranceDeg = 1.0;
    public static final double kTurnRateToleranceDegPerS = 1.0;
  }
}
