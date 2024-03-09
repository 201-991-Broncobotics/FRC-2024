package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.autonomous.*;

import frc.robot.commands.activatedCommands.*;
import frc.robot.commands.defaultCommands.*;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.Buttons.*;
import static frc.robot.Constants.TeleopSwerveConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver_XBox = new XboxController(driver_usb_port);
    private final XboxController operator = new XboxController(operator_usb_port);
    private final Joystick driver_TFlightHotasOne = new Joystick(joystick_usb_port);

    /* Both Controllers */
    private final Trigger terminateCommands = new JoystickButton(driver_XBox, xBoxTerminateCommandsDriverButton).or(new JoystickButton(operator, terminateCommandsOperatorButton)).or(new JoystickButton(driver_TFlightHotasOne, joystickTerminateCommandsButton));

    /* Driver Buttons */
    private final Trigger zeroGyro = new JoystickButton(driver_XBox, xBoxZeroGyroButton).or(new JoystickButton(driver_TFlightHotasOne, joystickZeroGyroButton));
    private final Trigger makeX = new JoystickButton(driver_XBox, xBoxMakeXButton).or(new JoystickButton(driver_TFlightHotasOne, joystickMakeXButton));

    private final Trigger robotCentric = new JoystickButton(driver_XBox, xBoxRobotCentricButton);

    /* Custom Triggers */

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Conveyor conveyor = new Conveyor();
    private final Flywheel flywheel = new Flywheel();
    private final Pivot pivot = new Pivot();
    private final Hang hang = new Hang();

    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        if (tFlightHotasOne_drive) {
            swerve.setDefaultCommand(
                new TeleopSwerveRelativeDirecting(
                    swerve, 
                    () -> -driver_TFlightHotasOne.getRawAxis(joystickTranslationAxis), 
                    () -> -driver_TFlightHotasOne.getRawAxis(joystickStrafeAxis), 
                    () -> -driver_TFlightHotasOne.getRawAxis(joystickRotationAxis), 
                    () -> false, 
                    () -> -driver_TFlightHotasOne.getPOV(), 
                    () -> {
                        if (driver_TFlightHotasOne.getRawButton(joystickSlowButton)) {
                            return teleop_swerve_slow_factor; // also sets direct angle I guess?
                        } else {
                            return 1.0;
                        }
                    }, // what we multiply translation speed by; rotation speed is NOT affected
                    () -> (Variables.bypass_rotation || driver_TFlightHotasOne.getRawButton(joystickDirectAngleButton))
                )
            );
        } else if (fancy_drive) {
            swerve.setDefaultCommand(
                new TeleopSwerveAbsoluteDirecting(
                    swerve, 
                    () -> -driver_XBox.getRawAxis(xBoxTranslationAxis), 
                    () -> -driver_XBox.getRawAxis(xBoxStrafeAxis), 
                    () -> driver_XBox.getRawAxis(xBoxDirectionXAxis), 
                    () -> -driver_XBox.getRawAxis(xBoxDirectionYAxis), 
                    () -> -driver_XBox.getPOV(), 
                    () -> driver_XBox.getRawAxis(xBoxTurnLeftAxis) - driver_XBox.getRawAxis(xBoxTurnRightAxis), 
                    () -> (driver_XBox.getRawButton(xBoxSlowButtonOne) || driver_XBox.getRawButton(xBoxSlowButtonTwo)), 
                    () -> (Variables.bypass_rotation || driver_XBox.getRawButton(1))
                )
            );
        } else {
            swerve.setDefaultCommand(
                new TeleopSwerveRelativeDirecting(
                    swerve, 
                    () -> -driver_XBox.getRawAxis(xBoxTranslationAxis), 
                    () -> -driver_XBox.getRawAxis(xBoxStrafeAxis), 
                    () -> -driver_XBox.getRawAxis(xBoxRotationAxis), 
                    () -> robotCentric.getAsBoolean(), 
                    () -> -driver_XBox.getPOV(), 
                    () -> 1 - 0.75 * driver_XBox.getRawAxis(xBoxSlowAxis), // what we multiply translation speed by; rotation speed is NOT affected
                    () -> (Variables.bypass_rotation || driver_XBox.getRawButton(1))
                )
            );
        }

        pivot.setDefaultCommand(
            new TeleopPivot(
                pivot, 
                () -> -driver_TFlightHotasOne.getRawAxis(2)
            )
        );

        hang.setDefaultCommand(
            new TeleopHang(
                hang, 
                () -> driver_TFlightHotasOne.getRawAxis(6)
            )
        );

        conveyor.setDefaultCommand(new DormantConveyor(conveyor));
        flywheel.setDefaultCommand(new DormantFlywheel(flywheel));
        intake.setDefaultCommand(new DormantIntake(intake));

        configureButtonBindings();
        configureNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
       /* Both Controllers */
        terminateCommands.toggleOnTrue(new InstantCommand());

        /* Driver Triggers */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro(0)));
        makeX.onTrue(new InstantCommand());

        new JoystickButton(driver_TFlightHotasOne, 5).toggleOnTrue(new IntakeCommand(pivot, intake, conveyor));
        new JoystickButton(driver_TFlightHotasOne, 14).toggleOnTrue(new FinishIntakeCommand(conveyor));
        new JoystickButton(driver_TFlightHotasOne, 6).toggleOnTrue(new OuttakeCommand(pivot, flywheel, conveyor));
        new JoystickButton(driver_TFlightHotasOne, 7).toggleOnTrue(new AmpCommand(pivot, flywheel, conveyor));
        new JoystickButton(driver_TFlightHotasOne, 8).toggleOnTrue(new StartingArmPosition(pivot));
        
        /* Operator Triggers */

        /* Custom Triggers */

        new JoystickButton(driver_TFlightHotasOne, 15).toggleOnTrue(new InstantCommand(() -> Variables.invert_rotation = !Variables.invert_rotation));
        new JoystickButton(driver_TFlightHotasOne, 13).toggleOnTrue(new InstantCommand(() -> Limelight.turnOffLED()));
    }

    public void configureNamedCommands() {
        NamedCommands.registerCommand("Shoot", new OuttakeCommand(pivot, flywheel, conveyor));
        NamedCommands.registerCommand("Intake", new IntakeCommand(pivot, intake, conveyor));
        NamedCommands.registerCommand("Amp", new AmpCommand(pivot, flywheel, conveyor));
        NamedCommands.registerCommand("Finish Intake", new FinishIntakeCommand(conveyor));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        // return Autonomous.getAutonomousCommand(swerve);
    }

    public void teleopInit() {
        swerve.teleopInit();
    }

    public Swerve getSwerve() {
        return swerve;
    }
}
