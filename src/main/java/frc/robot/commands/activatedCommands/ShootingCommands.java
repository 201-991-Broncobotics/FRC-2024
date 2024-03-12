package frc.robot.commands.activatedCommands;

import static frc.robot.Constants.TuningConstants.amp_angle;
import static frc.robot.Constants.TuningConstants.pivot_guard_angle;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Variables;
import frc.robot.commands.subcommands.*;
import frc.robot.subsystems.*;

public class ShootingCommands {

    public static Command amp(Pivot pivot, Flywheel flywheel, Conveyor conveyor) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                FlywheelCommands.amp(flywheel), 
                new SetArmPosition(pivot, amp_angle - pivot_guard_angle)
            ), new ParallelDeadlineGroup(
                new WaitCommand(3), 
                Commands.runOnce(conveyor::amp, conveyor), 
                new StabilizeArm(pivot, false)
            ), 
            new InstantCommand(() -> conveyor.stop()), 
            new InstantCommand(() -> flywheel.stop())
        );
    }

    public static Command speaker(Pivot pivot, Flywheel flywheel, Conveyor conveyor) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> Variables.bypass_rotation = true), 
                new ParallelCommandGroup(
                    new InstantCommand(() -> Variables.bypass_angling = true), 
                    Commands.waitUntil(pivot::pidCloseEnough)
                ), 
                FlywheelCommands.outtake(flywheel)
            ), new ParallelDeadlineGroup(
                new WaitCommand(3), 
                Commands.runOnce(conveyor::outtake, conveyor), 
                new StabilizeArm(pivot, false)
            ), 
            new InstantCommand(() -> Variables.bypass_rotation = false), 
            new InstantCommand(() -> Variables.bypass_angling = false), 
            new InstantCommand(() -> conveyor.stop()), 
            new InstantCommand(() -> flywheel.stop())
        );
    }
}
