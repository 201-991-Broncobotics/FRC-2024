package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.activatedCommands.IntakeCommand;
import frc.robot.commands.utilCommands.AutomaticDriveCommand;
import frc.robot.subsystems.*;

public class AutonomousIntake extends SequentialCommandGroup { // lmao

    public AutonomousIntake(Swerve swerve, Pivot pivot, Intake intake, Conveyor conveyor) {
        addRequirements(swerve, pivot, intake, conveyor);

        addCommands(
            new InstantCommand(() -> swerve.setTargetHeading(0)), 
            new ParallelRaceGroup(
                new WaitCommand(4), 
                new ParallelDeadlineGroup(
                    new IntakeCommand(pivot, intake, conveyor), 
                    new AutomaticDriveCommand(swerve, 1, 0.2)
                )
            ), new ParallelRaceGroup(
                new WaitCommand(4), 
                new AutomaticDriveCommand(swerve, -1, 0.2)
            )
        );
    }
}
