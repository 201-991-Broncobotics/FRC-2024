ackage frc.robot.autonomous;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Variables;
import frc.robot.Constants.TuningConstants;
import frc.robot.commands.subcommands.SetArmPosition;
import frc.robot.subsystems.*;

public class AutonomousCommands {
  public static void configureNamedCommands(Swerve swerve, Pivot pivot, Intake intake, Conveyor conveyor, Flywheel flywheel) {

    NamedCommands.registerCommand("intake", intake(pivot, intake, conveyor, flywheel));
    NamedCommands.registerCommand("stopIntaking", stopIntaking(intake, conveyor, flywheel));

    NamedCommands.registerCommand("shootWhenReady", shootWhenReady(conveyor, swerve, flywheel, pivot));
    NamedCommands.registerCommand("afterShoot", afterShoot(flywheel, conveyor));

    NamedCommands.registerCommand("pivotUnderStage", pivotUnderStage(pivot));
  }

  static Command afterShoot(Flywheel flywheel, Conveyor conveyor) {
    // we want to get rid of a note if we still have one
    var throwAwayNote = Commands.run(() -> {conveyor.outtake(); flywheel.outtake();}, conveyor, flywheel)
      .deadlineWith(Commands.waitSeconds(.5));

    var cleanup = Commands.run(() -> {
      conveyor.stop(); 
      flywheel.stop(); 
      Variables.bypass_angling = false; 
    }, conveyor, flywheel);

    return throwAwayNote.andThen(cleanup);
  }


  static Command intake(Pivot pivot, Intake intake, Conveyor conveyor, Flywheel flywheel) {
    return new SetArmPosition(pivot, TuningConstants.intake_angle).andThen(() -> {
      intake.intake();
      conveyor.intake();
      flywheel.intake();
    }, 
    intake, conveyor, flywheel);
  }

  static Command stopIntaking(Intake intake, Conveyor conveyor, Flywheel flywheel) {
    return Commands.parallel(
      Commands.run(intake::stop, intake),
      Commands.run(conveyor::stop, conveyor),
      Commands.run(flywheel::stop, flywheel)
    )
    .andThen(Commands.startEnd(conveyor::retract, conveyor::stop, conveyor).deadlineWith(Commands.waitSeconds(.25)));
  }

  static Command shootWhenReady(Conveyor conveyor, Swerve swerve, Flywheel flywheel, Pivot pivot) {
    // maybe wait until ChassisSpeeds = 0???
    var prepare = Commands.runOnce(() -> {Variables.bypass_angling = true; flywheel.outtake();}, flywheel);    

    return prepare.andThen(
      Commands.waitUntil(
        () -> Math.abs(swerve.getHeading().minus(ShootingMath.drivetrainAngle()).getDegrees()) < 1 && pivot.pidCloseEnough() && flywheel.isAtSpeed()
      ).andThen(conveyor::outtake, conveyor));
  }

  static Command pivotUnderStage(Pivot pivot) {
    return new SetArmPosition(pivot, TuningConstants.starting_angle);
  }
}
