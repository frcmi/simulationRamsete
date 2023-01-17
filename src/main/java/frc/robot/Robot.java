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
    private final RobotContainer container = new RobotContainer();

    @Override
    public void robotInit()
    {
        // Flush NetworkTables every loop. This ensures that robot pose and other values
        // are sent during every iteration.
        setNetworkTablesFlushEnabled(true);

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
