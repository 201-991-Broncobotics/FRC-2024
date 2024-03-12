package frc.robot.commands.activatedCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Flywheel;

import static frc.robot.Constants.TuningConstants.*;

public class FlywheelCommands {

    public static Command off(Flywheel flywheel) {
        return new InstantCommand(() -> flywheel.stop());
    }

    public static Command outtake(Flywheel flywheel) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> flywheel.outtake(), flywheel), 
            new ParallelRaceGroup(
                new WaitCommand(max_flywheel_acceleration_time), 
                Commands.waitUntil(flywheel::isAtSpeed)
            )
        );
    }

    public static Command amp(Flywheel flywheel) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> flywheel.amp(), flywheel), 
            new ParallelRaceGroup(
                new WaitCommand(max_flywheel_acceleration_time), 
                Commands.waitUntil(flywheel::isAtSpeed)
            )
        );
    }
}
