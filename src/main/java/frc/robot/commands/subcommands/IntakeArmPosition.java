package frc.robot.commands.subcommands;

import static frc.robot.Constants.TuningConstants.*;

import frc.robot.subsystems.Pivot;

public class IntakeArmPosition extends SetArmPosition {
    
    public IntakeArmPosition(Pivot pivot) {
        super(pivot, intake_angle);
    }

}
