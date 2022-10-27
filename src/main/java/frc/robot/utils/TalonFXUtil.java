package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainTalonFXSubsystem;

public class TalonFXUtil {

    private static final int CountsPerRevolution = 2048;

    // Helper methods to convert between native units and meters
    public static int distanceToNativeUnits(double positionInMeters) {
        double wheelRotations = positionInMeters/DriveConstants.kWheelCircumference;
        double  motorRotations = wheelRotations*DriveConstants.kGearRatio;
        int sensorCounts = (int)(motorRotations*CountsPerRevolution);

        return sensorCounts;
    }

    public static int velocityToNativeUnits (double velocityMetersPerSecond) {
        double wheelRotationsPerSecond = velocityMetersPerSecond/DriveConstants.kWheelCircumference;
        double motorRotationsPerSecond = wheelRotationsPerSecond*DriveConstants.kGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / 10;
        int sensorCountsPer100ms = (int)(motorRotationsPer100ms / CountsPerRevolution);
        return sensorCountsPer100ms;
    }
    
    public static double nativeUnitsToDistanceMeters(double sensorCounts) {
        double motorRotations = sensorCounts / CountsPerRevolution;
        double wheelRotations = motorRotations / DriveConstants.kGearRatio;
        double positionInMeters = wheelRotations * DriveConstants.kWheelCircumference;
        return positionInMeters;
    }

}