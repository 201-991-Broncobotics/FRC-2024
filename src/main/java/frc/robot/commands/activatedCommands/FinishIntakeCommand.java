package frc.robot.commands.activatedCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.utilCommands.CustomWaitCommand;
import frc.robot.subsystems.*;

import static frc.robot.Constants.TuningConstants.*;

public class FinishIntakeCommand extends SequentialCommandGroup {

    public FinishIntakeCommand(Conveyor conveyor) {
        addRequirements(conveyor);

        addCommands(
            /* new InstantCommand(() -> conveyor.intake()), 
            new CustomWaitCommand(add_conveyor_time, () -> conveyor.stop()), 
            new InstantCommand(() -> conveyor.stop()), 
            new WaitCommand(0.2), */ // because it decided to not work for no reason???
            new InstantCommand(() -> conveyor.retract()), 
            new CustomWaitCommand(retract_conveyor_time, () -> conveyor.stop()), 
            new InstantCommand(() -> conveyor.stop())
        );
    }

}