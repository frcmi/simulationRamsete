package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainTalonFXSubsystem;

import java.util.function.DoubleSupplier;


public class JoystickDriveCommand extends CommandBase {
    private final DrivetrainTalonFXSubsystem drivetrainTalonFXSubsystem;
    private DoubleSupplier joyX;
    private DoubleSupplier joyY;
    private DoubleSupplier joyZ;

    public JoystickDriveCommand(DrivetrainTalonFXSubsystem drivetrainTalonFXSubsystem,
                                DoubleSupplier joyY,
                                DoubleSupplier joyX,
                                DoubleSupplier joyZ) {
        this.drivetrainTalonFXSubsystem = drivetrainTalonFXSubsystem;
        this.joyY = joyY;
        this.joyX = joyX;
        this.joyZ = () -> 1.0;
        addRequirements(this.drivetrainTalonFXSubsystem);
    }


    @Override
    public void initialize() {
        System.out.println("INFO: JoystickDriveCommand \"initialize\"");
    }

    @Override
    public void execute() {
        drivetrainTalonFXSubsystem.arcDrive(joyY.getAsDouble(), joyX.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("INFO: JoystickDriveCommand \"end\": interrupted = %b%n", interrupted);
    }
}
