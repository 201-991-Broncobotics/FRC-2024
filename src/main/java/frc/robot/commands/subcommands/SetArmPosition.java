package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class SetArmPosition extends Command {
    private Pivot pivot;

    private final double target_angle;

    public SetArmPosition(Pivot pivot, double target_angle) {
        this.pivot = pivot;
        addRequirements(pivot);

        this.target_angle = target_angle;
    }

    @Override
    public void initialize() {
        pivot.brake();
        pivot.setTarget(target_angle);
    }

    @Override
    public boolean isFinished() {
        return pivot.pidCloseEnough();
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
