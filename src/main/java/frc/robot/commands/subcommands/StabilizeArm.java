package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class StabilizeArm extends Command {

    private Pivot pivot;

    private final boolean reset_target;

    public StabilizeArm(Pivot pivot, boolean reset_target) {
        this.pivot = pivot;
        addRequirements(pivot);

        this.reset_target = reset_target;
    }

    @Override
    public void initialize() {
        pivot.brake();

        if (reset_target) {
            pivot.resetTarget();
        }
    }

    @Override
    public void execute() {
        pivot.pidPower();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            pivot.brake();
            pivot.resetTarget();
        }
    }
}
