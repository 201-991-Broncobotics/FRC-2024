package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.subcommands.SetArmPosition;
import frc.robot.subsystems.*;

public class ShootingCommands {
  public static Command amp(Pivot pivot, Flywheel flywheel, Conveyor conveyor) {
    var start = new ParallelCommandGroup(FlywheelCommands.amp(flywheel), PivotCommands.toAmpPosition(pivot));
    var waits = new ParallelCommandGroup(PivotCommands.waitUntilAtPosition(pivot), FlywheelCommands.waitUntilAtSpeed(flywheel));
    var input = new ParallelDeadlineGroup(new WaitCommand(3), Commands.runOnce(conveyor::amp, conveyor));

    return new SequentialCommandGroup(start, waits, input);

  }

  public static Command speaker(Pivot pivot, Flywheel flywheel, Conveyor conveyor) {
    // TODO: auto aim
    var start = new ParallelCommandGroup(FlywheelCommands.outtake(flywheel), new SetArmPosition(pivot, 40));
    var waits = new ParallelCommandGroup(FlywheelCommands.waitUntilAtSpeed(flywheel), PivotCommands.waitUntilAtPosition(pivot));
    var input = new ParallelDeadlineGroup(new WaitCommand(3), Commands.runOnce(conveyor::outtake, conveyor));

    return new SequentialCommandGroup(start, waits, input);
  }
}
