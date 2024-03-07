package frc.robot.commands.activatedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.subcommands.*;

import frc.robot.subsystems.*;

public class AmpCommand extends SequentialCommandGroup {

    public AmpCommand(Pivot pivot, Flywheel flywheel, Conveyor conveyor) {
        addRequirements(pivot, flywheel, conveyor);

        addCommands(
            new AmpArmPosition(pivot),
            new Amp_Subcommand(flywheel, conveyor)
        );
    }

}