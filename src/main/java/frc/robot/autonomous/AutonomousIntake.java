package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.activatedCommands.IntakeCommand;
import frc.robot.subsystems.*;

public class AutonomousIntake extends SequentialCommandGroup { // lmao

    public AutonomousIntake(Swerve swerve, Pivot pivot, Intake intake, Conveyor conveyor) {
        addRequirements(swerve, pivot, intake, conveyor);

        addCommands(
            new ParallelDeadlineGroup(
                new WaitCommand(4), 
                new IntakeCommand(pivot, intake, conveyor)
                // also need a command to drive forward like 0.4 m
            )
        );
    }
    
}
