package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.activatedCommands.IntakeCommand;
import frc.robot.subsystems.*;

public class AutonomousIntake extends SequentialCommandGroup { // lmao

    public AutonomousIntake(Swerve swerve, Pivot pivot, Intake intake, Conveyor conveyor) {
        addRequirements(pivot, intake, conveyor);

        addCommands(
            new ParallelRaceGroup(
                new WaitCommand(4), 
                new IntakeCommand(pivot, intake, conveyor)
            )
        );
    }
}
