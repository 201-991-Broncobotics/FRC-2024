package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.activatedCommands.ShootingCommands;
import frc.robot.subsystems.*;

public class AutonomousOuttake extends ParallelRaceGroup {

    public AutonomousOuttake(Swerve swerve, Pivot pivot, Conveyor conveyor, Flywheel flywheel) {

        addCommands( // angle towards outtake
            new WaitCommand(6), 
            ShootingCommands.autonomousSpeaker(swerve, pivot, flywheel, conveyor)
        );
    }
    
}
