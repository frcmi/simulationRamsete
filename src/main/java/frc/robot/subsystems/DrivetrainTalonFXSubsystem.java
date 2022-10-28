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


    /**  -------- Coding an Archade Drive Train --------   
     * 
     * 1) Create Motor Objects for each motor. Pass in port # 
     * ex: private [Motor Model] [Motor Name] = new [Motor Model]([Port #]); 
     * 
     * 2) Create Pigeon. Pass in 1
     * ex: WPI_Pigeon2 [Pigeon Name] = new WPI_Pigeon2(1);
     * 
     * 3) Create field 
     * ex: private Field2d [Field Name] = new Field2d(); 
     * 
     * 4) create odometry differential Drive and drive Simulation
     * ex: private DifferentialDriveOdometry [name] = new DifferentialDriveOdometry(new Rotation2d()); 
     * ex: private DifferentialDrive [name] = new DifferentialDrive(leftMaster, rightMaster);
     * ex: private DrivebaseSimFX [name] = new DrivebaseSimFX(leftMaster, rightMaster, pidgey);
     * 
     * 5) configure, bind, and set spin in constructor
     * costructor: public DrivetrainTalonFXSubsystem() {} 
     * config motors. do to all motors: [motor name].configFactoryDefault(); 
     * bind motors. make left and right: [slave motor name].follow([master motor name]);
     * spin. left should be counter clockwise and vise versa:
     * spin: [master motor name].setInverted(TalonFXInvertType.[direction]);
     * spin: [slave motor name].setInverted(InvertType.FollowMaster);
     * 
     * 6) paste these in said constructor: 
     *  [Diffrential drive name].setSafetyEnabled(false);
     *  SmartDashboard.putData("Field", field);
     * */ 

    WPI_Pigeon2 pige = new WPI_Pigeon2(1); 

    private Field2d plane = new Field2d();
    
    private WPI_TalonFX leftM = new WPI_TalonFX(5); 
    private WPI_TalonFX leftS = new WPI_TalonFX(4); 
    private WPI_TalonFX rightM = new WPI_TalonFX(2); 
    private WPI_TalonFX rightS = new WPI_TalonFX(3); 

    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d()); 
    private DifferentialDrive dDrive = new DifferentialDrive(leftM, rightM);
    private DrivebaseSimFX driveSim = new DrivebaseSimFX(leftM, rightM, pige);
    
    public DrivetrainTalonFXSubsystem() { 

        SmartDashboard.putData("Field", plane); 
        dDrive.setSafetyEnabled(false);

        leftS.follow(leftM);
        rightS.follow(rightM); 

        leftM.setInverted(TalonFXInvertType.CounterClockwise);
        rightM.setInverted(TalonFXInvertType.Clockwise);
        leftS.setInverted(InvertType.FollowMaster);
        rightS.setInverted(InvertType.FollowMaster);

    }

    public void arcDrive(double xSpeed, double zRotation) {

        dDrive.arcadeDrive(xSpeed, zRotation);

    }

    public void reset() {

        leftM.setSelectedSensorPosition(0.0);
        rightM.setSelectedSensorPosition(0.0);

    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {

        leftM.setVoltage(leftVolts);
        rightM.setVoltage(rightVolts); 
        dDrive.feed();

    }

    //left is left right is right no caps or spaces will defualt to right with no input
    public double getVelocity(String side) {

        if (side == "left") {

            return leftM.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerTick * 10; 

        }
        else {

            return rightM.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerTick * 10; 

        }

    }

    public double getAverageDistance() {

        double leftMeters = TalonFXUtil.nativeUnitsToDistanceMeters(leftM.getSelectedSensorPosition());
        double rightMeters = TalonFXUtil.nativeUnitsToDistanceMeters(rightM.getSelectedSensorPosition());
        System.out.printf("INFO: Left: %.02f, Right: %.02f%n", leftMeters, rightMeters);
        System.out.printf("      Average: %.02f%n", (leftMeters + rightMeters) * 0.5);
        return (leftMeters + rightMeters) * 0.5;

    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {

        return new DifferentialDriveWheelSpeeds(getVelocity("left"), getVelocity("right"));

    }

    public void updateOdometry() {
        odometry.update(pige.getRotation2d(),
                TalonFXUtil.nativeUnitsToDistanceMeters(leftM.getSelectedSensorPosition()),
                TalonFXUtil.nativeUnitsToDistanceMeters(rightM.getSelectedSensorPosition()));
    }

    public void updateField() {
        plane.setRobotPose(odometry.getPoseMeters());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
      }

    public void resetOdometry(Pose2d pose) {
        reset();
        odometry.resetPosition(pose, pige.getRotation2d());
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