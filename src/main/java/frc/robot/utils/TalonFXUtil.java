package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DrivetrainTalonFXSubsystem;

public class TalonFXUtil {

    private static final int kCountsPerRev = 2048;
    private static final int k100msPerSecond = 10;

    // Helper methods to convert between meters and native units
    public static int distanceToNativeUnits(double positionMeters){
        double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(DrivetrainTalonFXSubsystem.kWheelRadiusInches));
        double motorRotations = wheelRotations * DrivetrainTalonFXSubsystem.kGearRatio;
        int sensorCounts = (int)(motorRotations * kCountsPerRev);
        return sensorCounts;
    }

    public static int velocityToNativeUnits(double velocityMetersPerSecond){
        double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(DrivetrainTalonFXSubsystem.kWheelRadiusInches));
        double motorRotationsPerSecond = wheelRotationsPerSecond * DrivetrainTalonFXSubsystem.kGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
        int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
        return sensorCountsPer100ms;
    }

    public static double nativeUnitsToDistanceMeters(double sensorCounts){
        double motorRotations = (double)sensorCounts / kCountsPerRev;
        double wheelRotations = motorRotations / DrivetrainTalonFXSubsystem.kGearRatio;
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DrivetrainTalonFXSubsystem.kWheelRadiusInches));
        return positionMeters;
    }

    public static double nativeUnitsToDistanceFeet(double sensorCounts){
        double motorRotations = (double)sensorCounts / kCountsPerRev;
        double wheelRotations = motorRotations / DrivetrainTalonFXSubsystem.kGearRatio;
        double positionMeters = wheelRotations * (2 * Math.PI * (DrivetrainTalonFXSubsystem.kWheelRadiusInches / 12.0));
        return positionMeters;
    }
}
