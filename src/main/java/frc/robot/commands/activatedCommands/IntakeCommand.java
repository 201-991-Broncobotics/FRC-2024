package frc.robot.commands.activatedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.subcommands.*;

import frc.robot.subsystems.*;

public class IntakeCommand extends SequentialCommandGroup {

    public IntakeCommand(Pivot pivot, Intake intake, Conveyor conveyor) {
        addRequirements(pivot, intake, conveyor);

        addCommands(
            new IntakeArmPosition(pivot), 
            new Intake_Subcommand(intake, conveyor)
        );
    }

}
