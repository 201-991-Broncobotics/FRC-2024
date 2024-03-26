// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.CTREConfigs;

import frc.robot.subsystems.Limelight;
import static frc.robot.Constants.GeneralConstants.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static final CTREConfigs ctreConfigs = new CTREConfigs();

    private Command autonomousCommand;

    private RobotContainer robotContainer;

    private double end_time = -1;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        Limelight.init();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        Variables.in_auto = false;
        Variables.in_teleop = false;
        log("TeleOp Time Left", 200);
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        Variables.in_auto = true;
        Variables.in_teleop = false;
        Limelight.setSide();
        log("TeleOp Time Left", 200);

        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousExit() {
        Variables.in_auto = false;
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    @Override
    public void teleopInit() {
        Variables.in_auto = false;
        Variables.in_teleop = true;
        Limelight.setSide();

        end_time = Timer.getFPGATimestamp() + 135;

        if (autonomousCommand != null) autonomousCommand.cancel();
        robotContainer.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        log("TeleOp Time Left", end_time - Timer.getFPGATimestamp());
    }

    @Override
    public void teleopExit() {
        Variables.in_teleop = false;
    }

    @Override
    public void testInit() {
        Variables.in_auto = false;
        Variables.in_teleop = false;

        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
}
