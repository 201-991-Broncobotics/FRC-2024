package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Variables;
import frc.robot.commands.activatedCommands.IntakeCommand;
import frc.robot.commands.utilCommands.LinearDriveCommand;
import frc.robot.subsystems.*;

public class AutonomousIntake extends SequentialCommandGroup { // lmao

    public AutonomousIntake(Swerve swerve, Pivot pivot, Intake intake, Conveyor conveyor) {
        addRequirements(swerve, pivot, intake, conveyor);

        addCommands(
            new InstantCommand(() -> swerve.setTargetHeading(Variables.isBlueAlliance ? 0 : 180)), 
            new ParallelRaceGroup(
                new WaitCommand(4), 
                new ParallelDeadlineGroup(
                    new IntakeCommand(pivot, intake, conveyor), 
                    new LinearDriveCommand(swerve, 1, 0.2)
                )
            ), new ParallelRaceGroup(
                new WaitCommand(4), 
                new LinearDriveCommand(swerve, -1, 0.2)
            )
        );
    }
}
