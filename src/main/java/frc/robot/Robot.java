// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.lang.reflect.Field;



public class Robot extends TimedRobot
{
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
    // to 1.
//    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
//    private final SlewRateLimiter rotateLimiter = new SlewRateLimiter(3);
//
//    private final Drivetrain drive = new Drivetrain();
//    private final RamseteController ramsete = new RamseteController();
//    private final Timer timer = new Timer();
//    private Trajectory trajectory;

    private final RobotContainer container = new RobotContainer();

    @Override
    public void robotInit()
    {
        // Flush NetworkTables every loop. This ensures that robot pose and other values
        // are sent during every iteration.
        setNetworkTablesFlushEnabled(true);
        
//        trajectory =
//                TrajectoryGenerator.generateTrajectory(
//                        new Pose2d(2, 2, new Rotation2d()),
//                        List.of(),
//                        new Pose2d(6, 4, new Rotation2d()),
//                        new TrajectoryConfig(2, 2));

        if (Robot.isSimulation()) {
            extendSimulationWatchdogPeriod();
        }
    }
    
    
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }
    
    
    @Override
    public void autonomousInit()
    {
        Command autoCommand = container.getAutonomousCommand();
        autoCommand.schedule();
    }
    
    
    @Override
    public void autonomousPeriodic()
    {
    }
    
    
    @Override
    @SuppressWarnings("LocalVariableName")
    public void teleopPeriodic()
    {
    }

    @Override
    public void simulationPeriodic()
    {
        container.simulationPeriodic();
    }

    private void extendSimulationWatchdogPeriod() {
        System.out.println("INFO: Extending IterativeRobot watchdog period");
        try {
            Field field = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            field.setAccessible(true);
            Watchdog watchdog = (Watchdog) field.get(this);
            watchdog.setTimeout(1.0);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            e.printStackTrace();
        }
    }
}
