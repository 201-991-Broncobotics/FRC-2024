package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.subcommands.SetArmPosition;
import frc.robot.subsystems.*;

public class AutonomousOuttake extends SequentialCommandGroup {

    public AutonomousOuttake(Swerve swerve, Pivot pivot, Conveyor conveyor, Flywheel flywheel) {
        addRequirements(swerve, pivot, conveyor, flywheel);

        addCommands( // angle towards outtake
            new ParallelRaceGroup(
                new WaitCommand(8), 
                new ParallelCommandGroup(
                    new SetArmPosition(pivot, ShootingMath.pivotAngle().getDegrees()), 
                    new InstantCommand(() -> swerve.targetSpeaker())
                    // also need a command to drive forward like 0.4 m
                    // also, this does not fucking work :skull: bc the default commands are not running (I'm pretty sure)
                )
            )
        );
    }
    
}
