// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.hang.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.limelight.*;
import frc.robot.subsystems.pivot.*;
import frc.robot.subsystems.shooter.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Pivot pivot;
  private final Shooter shooter;
  private final Limelight limelight;
  private final Hang hang;

  // Controller
  private final GenericHID driver = new GenericHID(0); // hotas
  private final CommandXboxController operator = new CommandXboxController(1); // xbox controller

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
            //   drive =
            // new Drive(
            //     new GyroIO() {},
            //     new ModuleIOSim(),
            //     new ModuleIOSim(),
            //     new ModuleIOSim(),
            //     new ModuleIOSim());
        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));

        intake = new Intake(new IntakeIOReal());
        pivot = new Pivot(new PivotIOReal());
        shooter = new Shooter(new ShooterIOReal());
        limelight = new Limelight(new LimelightIOReal(), drive);
        hang = new Hang(new HangIOReal());
        

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        intake = new Intake(new IntakeIOSim());
        pivot = new Pivot(new PivotIOSim());
        shooter = new Shooter(new ShooterIOSim());
        limelight = new Limelight(new LimelightIO() {}, drive);
        hang = new Hang(new HangIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        intake = new Intake(new IntakeIO() {});
        pivot = new Pivot(new PivotIO() {});
        shooter = new Shooter(new ShooterIO() {});
        limelight = new Limelight(new LimelightIO() {}, drive);
        hang = new Hang(new HangIO() {});

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
    configureNamedCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getRawAxis(1),
            () -> -driver.getRawAxis(0),
            () -> -driver.getRawAxis(5)));

    var stopWithXButton = new Trigger(() -> driver.getRawButton(2));
    stopWithXButton.onTrue(Commands.runOnce(drive::stopWithX, drive));

    operator.a().toggleOnTrue(new SmartIntake(shooter, intake));
    operator.b().toggleOnTrue(Commands.runOnce(intake::off, intake));
    operator.rightBumper().toggleOnTrue(Commands.runOnce(shooter::shootersOn, shooter));
    operator.leftBumper().toggleOnTrue(Commands.runOnce(shooter::shootersOff, shooter));
    operator.x().toggleOnTrue(Commands.runOnce(shooter::conveyorOn, shooter));
    operator.y().toggleOnTrue(Commands.runOnce(shooter::conveyorOff, shooter));

    pivot.setDefaultCommand(
        PivotCommands.basicOperatorControl(
            pivot, () -> MathUtil.applyDeadband(operator.getRightX() - operator.getLeftX(), 0.1)));

    hang.setDefaultCommand(hang.operatorControl(() -> 
      operator.getRightTriggerAxis() - operator.getLeftTriggerAxis()));
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("SmartIntake", new SmartIntake(shooter, intake));
    NamedCommands.registerCommand("PivotToIntake", PivotCommands.goToIntake(pivot));
    // todo: make this like SmartIntake
    NamedCommands.registerCommand("SmartShoot", Commands.startEnd(
      () -> {shooter.shootersOn(); shooter.conveyorOn();}, 
      () -> {shooter.shootersOff(); shooter.conveyorOff();},
      shooter
    ).withTimeout(1.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
