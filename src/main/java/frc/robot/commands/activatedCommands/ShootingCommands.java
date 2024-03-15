package frc.robot.commands.activatedCommands;

import static frc.robot.Constants.TuningConstants.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Variables;
import frc.robot.commands.subcommands.*;
import frc.robot.subsystems.*;

public class ShootingCommands {

    public static Command amp(Pivot pivot) {
        return new SetArmPosition(pivot, amp_angle - pivot_guard_angle);
    }

    public static Command autonomousSpeaker(Pivot pivot, Flywheel flywheel, Conveyor conveyor) {
        return new SequentialCommandGroup(

            new InstantCommand(() -> {}, flywheel, conveyor), 

            new ParallelCommandGroup(
                new InstantCommand(() -> Variables.bypass_rotation = true), 
                new SequentialCommandGroup(
                    new InstantCommand(() -> Variables.bypass_angling = true), 
                    Commands.waitUntil(pivot::pidCloseEnough)
                ), 
                FlywheelCommands.outtake(flywheel)
            ), 

            new InstantCommand(() -> conveyor.outtake()), 
            new WaitCommand(3), 

            new InstantCommand(() -> {
                Variables.bypass_rotation = false;
                Variables.bypass_angling = false;
                conveyor.stop();
                flywheel.stop();
            })
        ).handleInterrupt(() -> {
                Variables.bypass_rotation = false;
                Variables.bypass_angling = false;
                conveyor.stop();
                flywheel.stop();
            }
        );
    }

    public static Command speaker(Pivot pivot, Flywheel flywheel) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {}, flywheel), 
            new InstantCommand(() -> { Variables.bypass_angling = true; }),
            FlywheelCommands.outtake(flywheel)
        ).handleInterrupt(() -> {
                flywheel.stop();
                Variables.bypass_angling = false;
            }
        );
    }
}
