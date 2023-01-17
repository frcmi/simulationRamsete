package frc.robot.commands.autonomous;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

/** A command that will turn the robot to the specified angle. */

public class TurnToAngle extends PIDCommand {
    /**
     * Turns to robot to the specified angle.
     * @param targetAngleDegrees The angle to turn to
     * @param drive              The drive subsystem to use
     */
    public TurnToAngle(double targetAngleDegrees, DriveSubsystem drive) {
        super(new PIDController(AutoConstants.kTurnP, AutoConstants.kTurnI, AutoConstants.kTurnD),
            drive::getHeading, targetAngleDegrees, output -> drive.arcadeDrive(0, output, false), drive);
        getController().enableContinuousInput(-180, 180);

        getController().setTolerance(AutoConstants.kTurnToleranceDeg, AutoConstants.kTurnRateToleranceDegPerS);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
