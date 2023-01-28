package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.PPTrajectoryCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AutoPathMerger {

    protected PathPlannerTrajectory firstPath;
    protected PathPlannerTrajectory secondPath;
    protected PathPlannerTrajectory thirdPath;
    protected SequentialCommandGroup allPathsTogether;

    public AutoPathMerger(DriveSubsystem driveSubsystem, String partOneFileName, boolean doesGrab, String parkingLocation) {
        firstPath = PathPlanner.loadPath(partOneFileName, new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        String partTwoFileName = secondFileNameDetermine(partOneFileName, doesGrab);
        secondPath = PathPlanner.loadPath(partTwoFileName, new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        String partThreeFileName = thirdFileNameDetermine(partTwoFileName, parkingLocation);
        thirdPath = PathPlanner.loadPath(partThreeFileName, new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        allPathsTogether = new SequentialCommandGroup(
            new PPTrajectoryCommand(driveSubsystem, firstPath, true).getPathWithEvents(), 
            new PPTrajectoryCommand(driveSubsystem, secondPath, false).getPathWithEvents(),
            new PPTrajectoryCommand(driveSubsystem, thirdPath, false).getPathWithEvents()
        );
    }

    public static String secondFileNameDetermine(String partOneFileName, boolean doesGrab) {
        String toBeReturned = "";
        if (doesGrab) {
            if (partOneFileName.equals("North Path Up Part 1") || partOneFileName.equals("West Path Up Part 1")) {
                toBeReturned = "Up Part 2";
            } else if (partOneFileName.equals("South Path Down Part 1") || partOneFileName.equals("West Path Down Part 1")) {
                toBeReturned = "Down Part 2";
            }
        } else if (!(doesGrab)) {
            if (partOneFileName.equals("North Path Up Part 1") || partOneFileName.equals("West Path Up Part 1")) {
                toBeReturned = "Up Part 2 Skip";
            } else if (partOneFileName.equals("South Path Down Part 1") || partOneFileName.equals("West Path Down Part 1")) {
                toBeReturned = "Down Part 2 Skip";
            }
        }
        return toBeReturned;
    }

    public static String thirdFileNameDetermine(String secondFileName, String parkingLocation) {
        String toBeReturned = "";
        if (secondFileName.equals("Up Part 2") || secondFileName.equals("Up Part 2 Skip")) {
            if (parkingLocation.equals("Upper")) {
                toBeReturned = "Up Part 3 Upper";
            } else if (parkingLocation.equals("Middle")) {
                toBeReturned = "Up Part 3 Middle";
            } else if (parkingLocation.equals("Lower")) {
                toBeReturned = "Up Part 3 Lower";
            }
        } else if (secondFileName.equals("Down Part 2") || secondFileName.equals("Down Part 2 Skip")) {
            if (parkingLocation.equals("Upper")) {
                toBeReturned = "Down Part 3 Upper";
            } else if (parkingLocation.equals("Middle")) {
                toBeReturned = "Down Part 3 Middle";
            } else if (parkingLocation.equals("Lower")) {
                toBeReturned = "Down Part 3 Lower";
            }
        }
        return toBeReturned;
    }

    public SequentialCommandGroup getMergedPathCommand() {
        return allPathsTogether;
    }

}