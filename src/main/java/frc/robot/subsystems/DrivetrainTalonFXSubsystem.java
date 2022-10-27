/**
 * Borrowed copiously from Phoenix-Example-Languages
 * Java Talon FX (Falcon 500) -> DifferentialDrive
 * https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages
 */

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
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

    private WPI_TalonFX leftLeader = new WPI_TalonFX(5);
    private WPI_TalonFX leftFollower = new WPI_TalonFX(4);
    private WPI_TalonFX rightLeader = new WPI_TalonFX(2);
    private WPI_TalonFX rightFollower= new WPI_TalonFX(3);

    WPI_Pigeon2 pidgeon2 = new WPI_Pigeon2(1);

    private Field2d field2d = new Field2d();
    private DifferentialDriveOdometry diffDriveOd = new DifferentialDriveOdometry(new Rotation2d());

    private DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader);

    private DrivebaseSimFX driveSim = new DrivebaseSimFX(leftFollower, rightLeader, pidgeon2);

    public DrivetrainTalonFXSubsystem() {
        // Configure motors
        leftLeader.configFactoryDefault();
        leftFollower.configFactoryDefault();
        rightLeader.configFactoryDefault();
        rightFollower.configFactoryDefault();

        // Set followers as followers
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        // Set rotation Clockwise (default) - Set rotation Counterclockwise (invert direction)
        leftLeader.setInverted(TalonFXInvertType.CounterClockwise);
        rightLeader.setInverted(TalonFXInvertType.Clockwise);

        leftFollower.setInverted(InvertType.FollowMaster);
        rightFollower.setInverted(InvertType.FollowMaster);

        diffDrive.setSafetyEnabled(false);

        SmartDashboard.putData("Field", field2d);
    }

    // Transfers values to diffDrive.ArcadeDrive(), which is a different function
    public void arcadeDrive(double xSpeed, double zRotation) {
        diffDrive.arcadeDrive(xSpeed, zRotation * 0.5);
    }

    // Set the voltage of the motors
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
        diffDrive.feed();
    }

    // Immediatly stop the drivetrain
    public void stop() {
        diffDrive.arcadeDrive(0.0, 0.0);
    }

    // Reset Sensors
    public void reset() {
        leftLeader.setSelectedSensorPosition(0.0);
        rightLeader.setSelectedSensorPosition(0.0);
    }

    public double getAverageDistance() {
        double lDistance = TalonFXUtil.nativeUnitsToDistanceMeters(leftLeader.getSelectedSensorPosition());
        double rDistance = TalonFXUtil.nativeUnitsToDistanceMeters(rightLeader.getSelectedSensorPosition());
        System.out.printf("INFO: Left: " + lDistance + " Right: " + rDistance);
        System.out.printf("      Average: " + (lDistance + rDistance)/2);
        return (lDistance + rDistance)/2;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            leftLeader.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerTick * 10,
            rightLeader.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerTick * 10);
    }

    public Pose2d getPose() {
        return diffDriveOd.getPoseMeters();
    }

    public void resetOdometry(Pose2d position) {
        reset();
        diffDriveOd.resetPosition(position, pidgeon2.getRotation2d());
    }

    @Override
    public void periodic() {
        // Update the odometry
        diffDriveOd.update(pidgeon2.getRotation2d(),
            TalonFXUtil.nativeUnitsToDistanceMeters(leftLeader.getSelectedSensorPosition()),
            TalonFXUtil.nativeUnitsToDistanceMeters(rightLeader.getSelectedSensorPosition()));

        // Update the field
        field2d.setRobotPose(diffDriveOd.getPoseMeters());
    }

    public void simulationPeriodic() {
        driveSim.run();
    }
} 