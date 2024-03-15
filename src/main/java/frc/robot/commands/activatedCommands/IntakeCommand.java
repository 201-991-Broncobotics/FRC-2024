package frc.robot.commands.activatedCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.subcommands.*;

import frc.robot.subsystems.*;

import static frc.robot.Constants.TuningConstants.*;

public class IntakeCommand extends SequentialCommandGroup {

    public IntakeCommand(Pivot pivot, Intake intake, Conveyor conveyor) {
        addRequirements(pivot, intake, conveyor);

        addCommands(
            new SetArmPosition(pivot, intake_angle), 
            new ParallelDeadlineGroup( // could also be ParallelRaceGroup
                new SequentialCommandGroup(
                    new Intake_Subcommand(intake, conveyor), // ParallelDeadlineGroup???
                    new WaitCommand(0.2), 
                    new RetractConveyor(conveyor)
                ).handleInterrupt(() -> new RetractConveyor(conveyor).schedule()),
                new StabilizeArm(pivot, true)
            )
        );
    }

}
