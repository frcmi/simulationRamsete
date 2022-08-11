package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainTalonFXSubsystem;


public class DriveForTimeCommand extends WaitCommand {
    private final DrivetrainTalonFXSubsystem drivetrainTalonFXSubsystem;
    private double speed;

    public DriveForTimeCommand(DrivetrainTalonFXSubsystem drivetrainTalonFXSubsystem,
                               double speed,
                               double time) {
        super(time);
        this.drivetrainTalonFXSubsystem = drivetrainTalonFXSubsystem;
        this.speed = speed;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.drivetrainTalonFXSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("INFO: DriveForTimeCommand \"initialize\"");
        super.initialize();
        drivetrainTalonFXSubsystem.arcadeDrive(speed, 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("INFO: DriveForTimeCommand \"end\": interrupted = %b%n", interrupted);
        super.end(interrupted);
        drivetrainTalonFXSubsystem.stop();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
