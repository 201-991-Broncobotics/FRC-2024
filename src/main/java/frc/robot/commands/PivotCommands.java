package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.subcommands.SetArmPosition;
import frc.robot.subsystems.Pivot;
import static frc.robot.Constants.TuningConstants.*;

public final class PivotCommands {
  public static Command toIntakePosition(Pivot pivot) {
      return new SetArmPosition(pivot, intake_angle);
  }
  public static Command toAmpPosition(Pivot pivot) {
      return new SetArmPosition(pivot, amp_angle - pivot_guard_angle);
  }
  public static Command waitUntilAtPosition(Pivot pivot) {
    return Commands.waitUntil(pivot::pidCloseEnough);
  }
}
