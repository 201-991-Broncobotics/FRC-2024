package frc.robot.autonomous;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Variables;
import frc.robot.Constants.TuningConstants;
import frc.robot.commands.defaultCommands.TeleopSwerveRelativeDirecting;
import frc.robot.commands.subcommands.SetArmPosition;
import frc.robot.subsystems.*;

public class AutonomousCommands {
  public static void configureNamedCommands(Swerve swerve, Pivot pivot, Intake intake, Conveyor conveyor, Flywheel flywheel) {

    NamedCommands.registerCommand("intake", intake(pivot, intake, conveyor, flywheel));
    NamedCommands.registerCommand("stopIntaking", stopIntaking(intake, conveyor, flywheel, pivot));

    NamedCommands.registerCommand("prepareShoot", prepareShoot(conveyor, swerve, flywheel, pivot));
    NamedCommands.registerCommand("shoot", shoot(swerve, conveyor, pivot, flywheel));
    NamedCommands.registerCommand("afterShoot", afterShoot(flywheel, conveyor));

    NamedCommands.registerCommand("pivotUnderStage", pivotUnderStage(pivot));
  }

  static Command afterShoot(Flywheel flywheel, Conveyor conveyor) {
    return Commands.runOnce(() -> {
        conveyor.stop(); 
        flywheel.stop(); 
        Variables.bypass_angling = false; 
      }, conveyor, flywheel);
  }


  static Command intake(Pivot pivot, Intake intake, Conveyor conveyor, Flywheel flywheel) {
    return Commands.parallel(
      Commands.run(intake::intake, intake),
      Commands.run(conveyor::intake, conveyor),
      Commands.run(flywheel::intake, flywheel),
      new SetArmPosition(pivot, TuningConstants.intake_angle).andThen(Commands.run(pivot::pidPower, pivot))
    );
  }

  static Command stopIntaking(Intake intake, Conveyor conveyor, Flywheel flywheel, Pivot pivot) {
    return Commands.parallel(
      Commands.runOnce(intake::stop, intake),
      Commands.runOnce(conveyor::stop, conveyor),
      Commands.runOnce(flywheel::stop, flywheel),
      Commands.runOnce(pivot::brake, pivot)
    )
    .andThen(Commands.waitSeconds(0.35).deadlineWith(
      Commands.startEnd(conveyor::retract, conveyor::stop, conveyor), 
      Commands.startEnd(flywheel::intake, flywheel::stop, flywheel))
    );
  }

  static Command prepareShoot(Conveyor conveyor, Swerve swerve, Flywheel flywheel, Pivot pivot) {
    var prepare = Commands.runOnce(() -> {Variables.bypass_angling = true; pivot.setTarget(ShootingMath.pivotAngle().getDegrees()); flywheel.autoOuttake();}, flywheel);    

    var whileWaiting = Commands.parallel(
        Commands.run(() -> {
          pivot.setTarget(ShootingMath.pivotAngle().getDegrees());
          pivot.pidPower();
        }, pivot)
    );

    return prepare.andThen(whileWaiting);
  }

  static Command pivotUnderStage(Pivot pivot) {
    return new SetArmPosition(pivot, TuningConstants.starting_angle).andThen(pivot::pidPower, pivot);
  }

  public static Command shoot(Swerve swerve, Conveyor conveyor, Pivot pivot, Flywheel flywheel) {
    var autoAimSwerve = Commands.runOnce(() -> { Variables.bypass_rotation = true; swerve.targetSpeaker(); })
          .andThen( 
              new ParallelDeadlineGroup(
                  Commands.waitUntil(swerve::pidCloseEnough), 
                  new ParallelRaceGroup(
                      new TeleopSwerveRelativeDirecting(swerve, () -> 0, () -> 0, () -> 0, () -> false, () -> -1, () -> 0.5, () -> true), 
                      Commands.waitUntil(swerve::pidCloseEnough)
              ).andThen(swerve::brake)
          ));

    return new SequentialCommandGroup(
            autoAimSwerve.onlyIf(() -> 
                Math.abs(ShootingMath.drivetrainAngle().minus(swerve.getGyroYaw()).getDegrees()) < 1.5),
            Commands.waitUntil(() -> pivot.pidCloseEnough() && flywheel.isAtSpeed()),
            Commands.waitSeconds(0.25).deadlineWith(Commands.run(conveyor::outtake, conveyor))
    ).raceWith(Commands.run(() -> {
      pivot.setTarget(ShootingMath.pivotAngle().getDegrees());
      pivot.pidPower();
      flywheel.autoOuttake();
    }, pivot, flywheel));
  }
}
