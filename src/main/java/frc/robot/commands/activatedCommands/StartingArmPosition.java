package frc.robot.commands.activatedCommands;

import static frc.robot.Constants.TuningConstants.*;

import frc.robot.commands.subcommands.SetArmPosition;

import frc.robot.subsystems.Pivot;

public class StartingArmPosition extends SetArmPosition {
    
    public StartingArmPosition(Pivot pivot) {
        super(pivot, starting_angle + pivot_guard_angle);
    }

}
