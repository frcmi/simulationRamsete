package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainTalonFXSubsystem;


public class DriveForDistanceCommand extends CommandBase {
    private final DrivetrainTalonFXSubsystem drivetrainTalonFXSubsystem;
    private final double speed;
    private final double targetDistance;

    private Double initialDistance = null;

    public DriveForDistanceCommand(DrivetrainTalonFXSubsystem drivetrainTalonFXSubsystem,
                                   double speed,
                                   double distance) {
        this.drivetrainTalonFXSubsystem = drivetrainTalonFXSubsystem;
        this.speed = speed;
        this.targetDistance = distance;
        addRequirements(this.drivetrainTalonFXSubsystem);
    }

    @Override
    public void initialize() {
        System.out.printf("INFO: DriveForDistanceCommand \"initialize\" (Target Distance: %.02f)%n", targetDistance);
        // drivetrainTalonFXSubsystem.reset();
        drivetrainTalonFXSubsystem.arcadeDrive(speed, 0.0);
    }

    @Override
    public void execute() {
        // No need to keep executing previous commands
    }

    @Override
    public boolean isFinished() {
        double dist = drivetrainTalonFXSubsystem.getAverageDistance();
        return dist >= targetDistance + initialDistance();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("INFO: DriveForDistanceCommand \"end\": interrupted = %b%n", interrupted);
        initialDistance = null;
        drivetrainTalonFXSubsystem.stop();
    }

    private double initialDistance() {
        if (initialDistance == null) {
            initialDistance = drivetrainTalonFXSubsystem.getAverageDistance();
            System.out.println("Getting Initial Distance.. " + initialDistance);
        }
        return initialDistance;
    }
}
