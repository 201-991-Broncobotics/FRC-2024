package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.activatedCommands.RetractConveyor;
import frc.robot.commands.subcommands.*;

import frc.robot.subsystems.*;

import static frc.robot.Constants.TuningConstants.*;

public class AutonomousIntake extends SequentialCommandGroup {

    public AutonomousIntake(Pivot pivot, Intake intake, Conveyor conveyor) {
        addRequirements(pivot, intake, conveyor);

        addCommands(
            new SetArmPosition(pivot, intake_angle), 
            new ParallelDeadlineGroup( // could also be ParallelRaceGroup
                new SequentialCommandGroup(
                    new Intake_Subcommand(intake, conveyor, true), // ParallelDeadlineGroup???
                    new WaitCommand(0.2),
                    new RetractConveyor(conveyor)
                ),
                new StabilizeArm(pivot, true)
            )
        );
    }

}
