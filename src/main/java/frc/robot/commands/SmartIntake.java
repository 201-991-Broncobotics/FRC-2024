package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class SmartIntake extends Command {
  Shooter shooter;
  Intake intake;
  // basically, the way we want this command to work is when the intake shoots to high voltage, then goes back down, we know we're done intaking. so, we keep track of the intake current on the previous periodic run
  double lastCurrent;
  // once the intake is done + extra time is done, we can stop the intake + conveyor
  double endTime;


  public SmartIntake(Shooter shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;

    addRequirements(shooter, intake);
  }

  @Override
  public void execute() {
    double newCurrent = intake.getCurrent();
    // if the current is <10 and our previous current was >10, we know we're done intaking
    if (newCurrent < 10 && lastCurrent > 10) {
      endTime = Timer.getFPGATimestamp() + IntakeConstants.intakeToConveyorTime;
    }
  }

  @Override
  public void initialize() {
    intake.on();
    shooter.conveyorOn();
    // not sure if we need this here, but whatever
    endTime = Double.MAX_VALUE;
    lastCurrent = 0.0;
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > endTime;
  }

  @Override
  public void end(boolean _interrupted) {
    intake.off();
    shooter.conveyorOff();
  }
}
