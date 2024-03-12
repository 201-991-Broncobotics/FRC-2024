package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Flywheel;

public class FlywheelCommands {
  public static Command waitUntilAtSpeed(Flywheel flywheel) {
    return Commands.waitUntil(flywheel::isAtSpeed);
  }
  public static Command off(Flywheel flywheel) {
    return Commands.run(flywheel::stop, flywheel);
  }
  public static Command outtake(Flywheel flywheel) {
    return Commands.run(flywheel::outtake, flywheel);
  }
  public static Command amp(Flywheel flywheel) {
    return Commands.run(flywheel::amp, flywheel);
  }
}
