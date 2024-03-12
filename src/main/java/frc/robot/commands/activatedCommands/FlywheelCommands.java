package frc.robot.commands.activatedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Flywheel;

public class FlywheelCommands {

    public static Command off(Flywheel flywheel) {
        return Commands.run(flywheel::stop, flywheel);
    }

    public static Command outtake(Flywheel flywheel) {
        return new SequentialCommandGroup(
            Commands.run(flywheel::outtake, flywheel), 
            Commands.waitUntil(flywheel::isAtSpeed)
        );
    }

    public static Command amp(Flywheel flywheel) {
        return new SequentialCommandGroup(
            Commands.run(flywheel::amp, flywheel), 
            Commands.waitUntil(flywheel::isAtSpeed)
        );
    }
}
