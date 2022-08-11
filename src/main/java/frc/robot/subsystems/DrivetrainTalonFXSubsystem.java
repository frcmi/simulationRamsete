/**
 * Borrowed copiously from Phoenix-Example-Languages
 * Java Talon FX (Falcon 500) -> DifferentialDrive
 * https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages
 */

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.DrivebaseSimFX;
import frc.robot.utils.TalonFXUtil;


public class DrivetrainTalonFXSubsystem extends SubsystemBase {

    private WPI_TalonFX leftMaster = new WPI_TalonFX(5);
    private WPI_TalonFX leftSlave = new WPI_TalonFX(4);
    private WPI_TalonFX rightMaster = new WPI_TalonFX(2);
    private WPI_TalonFX rightSlave = new WPI_TalonFX(3);

    WPI_Pigeon2 pidgey = new WPI_Pigeon2(1);

    private Field2d field = new Field2d();
    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d());

    private DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

    public static final double kWheelRadiusInches = 3;
    public static final double kGearRatio = 11.64;

    private DrivebaseSimFX driveSim = new DrivebaseSimFX(leftMaster, rightMaster, pidgey);

    public DrivetrainTalonFXSubsystem() {
        leftMaster.configFactoryDefault();
        leftSlave.configFactoryDefault();
        rightMaster.configFactoryDefault();
        rightSlave.configFactoryDefault();

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        // Spin counterclockwise (default) - Spin Clockwise (invert direction)
        leftMaster.setInverted(TalonFXInvertType.CounterClockwise);
        rightMaster.setInverted(TalonFXInvertType.Clockwise);

        leftSlave.setInverted(InvertType.FollowMaster);
        rightSlave.setInverted(InvertType.FollowMaster);

        diffDrive.setSafetyEnabled(false);

        SmartDashboard.putData("Field", field);
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        diffDrive.arcadeDrive(xSpeed, zRotation);
    }

    public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
        diffDrive.arcadeDrive(xSpeed, zRotation, squareInputs);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMaster.setVoltage(leftVolts);
        rightMaster.setVoltage(rightVolts);
        diffDrive.feed();
    }

    public void stop() {
        diffDrive.arcadeDrive(0.0, 0.0);
    }

    public void reset() {
        leftMaster.setSelectedSensorPosition(0.0);
        rightMaster.setSelectedSensorPosition(0.0);
    }

    public double getAverageDistance() {
        double leftFeet = TalonFXUtil.nativeUnitsToDistanceFeet(leftMaster.getSelectedSensorPosition());
        double rightFeet = TalonFXUtil.nativeUnitsToDistanceFeet(rightMaster.getSelectedSensorPosition());
        System.out.printf("INFO: Left: %.02f, Right: %.02f%n", leftFeet, rightFeet);
        System.out.printf("      Average: %.02f%n", (leftFeet + rightFeet) * 0.5);
        return (leftFeet + rightFeet) * 0.5;
    }

      // Get the Left Wheel Velocity of the Robot in Meters per Second
    public double getLeftVelocity() {
        return leftMaster.getSelectedSensorVelocity() * Units.inchesToMeters(DriveConstants.kWheelCircumference) / 60;
    }

    // Get the Right Wheel Velocity of the Robot in Meters per Second
    public double getRightVelocity() {
        return rightMaster.getSelectedSensorVelocity() * Units.inchesToMeters(DriveConstants.kWheelCircumference) / 60;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    public void updateOdometry() {
        odometry.update(pidgey.getRotation2d(),
                TalonFXUtil.nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorPosition()),
                TalonFXUtil.nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorPosition()));
    }

    public void updateField() {
        field.setRobotPose(getPose());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
      }

    public void resetOdometry(Pose2d pose) {
        reset();
        odometry.resetPosition(pose, pidgey.getRotation2d());
      }

    @Override
    public void periodic() {
        updateOdometry();
        updateField();
    }

    public void simulationPeriodic() {
        driveSim.run();
    }
}

