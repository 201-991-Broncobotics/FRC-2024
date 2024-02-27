package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class SmartShoot extends Command {
  Shooter shooter;
  double lastCurrent = 0.0;
  boolean shouldEnd = false;

  public SmartShoot(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    lastCurrent = 0.0;
    shouldEnd = false;
    shooter.shootersOn();
    shooter.conveyorOn();
  }

  @Override
  public void execute() {
    double current = shooter.getAverageShooterCurrent();
    if (current < 10 && lastCurrent > 10) {
      shouldEnd = true;
    }
  }

  @Override
  public boolean isFinished() {
    return shouldEnd;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.shootersOff();
    shooter.conveyorOff();
  }

}
