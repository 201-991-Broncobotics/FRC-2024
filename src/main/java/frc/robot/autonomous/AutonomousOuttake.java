package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.activatedCommands.ShootingCommands;
import frc.robot.commands.defaultCommands.TeleopSwerveRelativeDirecting;
import frc.robot.subsystems.*;

public class AutonomousOuttake extends SequentialCommandGroup {

    public AutonomousOuttake(Swerve swerve, Pivot pivot, Conveyor conveyor, Flywheel flywheel) {
        addRequirements(swerve, pivot, conveyor, flywheel);

        addCommands( // angle towards outtake
            new ParallelDeadlineGroup(
                new ParallelRaceGroup(
                    new WaitCommand(8), 
                    ShootingCommands.speaker(pivot, flywheel, conveyor)
                ), 
                new TeleopSwerveRelativeDirecting(swerve, () -> 0, () -> 0, () -> 0, () -> false, () -> -1, () -> 1, () -> true)
            )
        );
    }
    
}
