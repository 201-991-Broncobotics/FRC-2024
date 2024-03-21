package frc.robot.autonomous;

import static frc.robot.Constants.GeneralConstants.log;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    var throwAwayNote = Commands.waitSeconds(0.25).deadlineWith(
      Commands.run(() -> {conveyor.outtake(); flywheel.outtake();}, conveyor, flywheel)
    );

    var cleanup = Commands.runOnce(() -> {
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
      pivot.pidPower();
    }, 
    intake, conveyor, flywheel, pivot);
  }

  static Command stopIntaking(Intake intake, Conveyor conveyor, Flywheel flywheel) {
    return Commands.parallel(
      Commands.run(intake::stop, intake),
      Commands.run(conveyor::stop, conveyor),
      Commands.run(flywheel::stop, flywheel)
    )
    .andThen(
      Commands.waitSeconds(.125).deadlineWith(Commands.startEnd(conveyor::retract, conveyor::stop, conveyor))
    );
  }

  static Command shootWhenReady(Conveyor conveyor, Swerve swerve, Flywheel flywheel, Pivot pivot) {
    var prepare = Commands.runOnce(() -> {Variables.bypass_angling = true; pivot.setTarget(ShootingMath.pivotAngle().getDegrees()); flywheel.outtake();}, flywheel);    

    var whileWaiting = Commands.parallel(
      Commands.run(pivot::pidPower, pivot)
    );

    // maybe wait until ChassisSpeeds = 0???
    var isReady = Commands.waitUntil(
        () -> Math.abs(swerve.getHeading().minus(ShootingMath.drivetrainAngle()).getDegrees()) < 2 && pivot.pidCloseEnough() && flywheel.isAtSpeed()
    ).andThen(() -> {log("SHOOTING RN", Timer.getFPGATimestamp());});

    var shoot = Commands.waitSeconds(0.5).deadlineWith(Commands.run(conveyor::outtake, conveyor));

    return new ParallelDeadlineGroup(new SequentialCommandGroup(prepare, isReady, shoot), whileWaiting);
  }

  static Command pivotUnderStage(Pivot pivot) {
    return new SetArmPosition(pivot, TuningConstants.starting_angle).andThen(pivot::pidPower, pivot);
  }
}
