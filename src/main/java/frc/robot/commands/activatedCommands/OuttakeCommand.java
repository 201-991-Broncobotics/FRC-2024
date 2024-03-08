package frc.robot.commands.activatedCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.subcommands.*;

import frc.robot.subsystems.*;

public class OuttakeCommand extends SequentialCommandGroup {

    public OuttakeCommand(Pivot pivot, Flywheel flywheel, Conveyor conveyor) {
        addRequirements(pivot, flywheel, conveyor);

        addCommands(
            new OuttakeArmPosition(pivot), 
            new ParallelDeadlineGroup( // could also be ParallelRaceGroup
                new Outtake_Subcommand(flywheel, conveyor), 
                new StabilizeArmCommand(pivot, false)
            )
        );
    }

}
