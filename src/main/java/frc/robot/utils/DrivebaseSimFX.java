package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DrivebaseSimFX {
    private WPI_TalonFX _leftMaster, _rightMaster;
    private WPI_Pigeon2 _pidgey;

    private TalonFXSimCollection _leftMasterSim, _rightMasterSim;
    private BasePigeonSimCollection _pidgeySim;


    //Simulation model of the drivetrain
    private DifferentialDrivetrainSim _driveSim = new DifferentialDrivetrainSim(
            DCMotor.getFalcon500(2),  //2 Falcon 500s on each side of the drivetrain.
            DriveConstants.kGearRatio,               //Standard AndyMark Gearing reduction.
            2.1,                      //MOI of 2.1 kg m^2 (from CAD model).
            26.5,                     //Mass of the robot is 26.5 kg.
            DriveConstants.kWheelDiameterMeters,  //Robot uses 3" radius (6" diameter) wheels.
            0.546,                    //Distance between wheels is _ meters.

            // The standard deviations for measurement noise:
            // x and y:          0.001 m
            // heading:          0.001 rad
            // l and r velocity: 0.1   m/s
            // l and r position: 0.005 m
            null //VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this line to add measurement noise.
    );

    /**
     * Creates a new drivebase simualtor using Falcon 500 motors.
     *
     * @param leftMaster the left master Falcon
     * @param rightMaster the right master Falcon
     * @param pidgey the Pigeon IMU
     */
    public DrivebaseSimFX(WPI_TalonFX leftMaster, WPI_TalonFX rightMaster, WPI_Pigeon2 pidgey) {
        _leftMaster = leftMaster;
        _rightMaster = rightMaster;
        _pidgey = pidgey;

        _leftMasterSim = leftMaster.getSimCollection();
        _rightMasterSim = rightMaster.getSimCollection();
        _pidgeySim = pidgey.getSimCollection();
    }

    /**
     * Runs the drivebase simulator.
     */
    public void run() {
        // Set the inputs to the system. Note that we need to use
        // the output voltage, NOT the percent output.
        _driveSim.setInputs(_leftMasterSim.getMotorOutputLeadVoltage(),
                -_rightMasterSim.getMotorOutputLeadVoltage()); //Right side is inverted, so forward is negative voltage

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        _driveSim.update(0.02);

        // Update all of our sensors.
        _leftMasterSim.setIntegratedSensorRawPosition(
                TalonFXUtil.distanceToNativeUnits(
                        _driveSim.getLeftPositionMeters()));
        _leftMasterSim.setIntegratedSensorVelocity(
                TalonFXUtil.velocityToNativeUnits(
                        _driveSim.getLeftVelocityMetersPerSecond()));
        _rightMasterSim.setIntegratedSensorRawPosition(
                TalonFXUtil.distanceToNativeUnits(
                        -_driveSim.getRightPositionMeters()));
        _rightMasterSim.setIntegratedSensorVelocity(
                TalonFXUtil.velocityToNativeUnits(
                        -_driveSim.getRightVelocityMetersPerSecond()));
        _pidgeySim.setRawHeading(_driveSim.getHeading().getDegrees());

        //Update other inputs to Talons
        _leftMasterSim.setBusVoltage(RobotController.getBatteryVoltage());
        _rightMasterSim.setBusVoltage(RobotController.getBatteryVoltage());
    }
}

