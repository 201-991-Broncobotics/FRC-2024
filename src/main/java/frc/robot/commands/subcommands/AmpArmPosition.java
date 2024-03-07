package frc.robot.commands.subcommands;

import static frc.robot.Constants.TuningConstants.*;

import frc.robot.subsystems.Pivot;

public class AmpArmPosition extends SetArmPosition {
    
    public AmpArmPosition(Pivot pivot) {
        super(pivot, amp_angle - pivot_guard_angle);
    }

}
