package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class SetArmPosition extends Command {
    private Pivot pivot;

    private final Rotation2d target_angle;

    public SetArmPosition(Pivot pivot, Rotation2d target_angle) {
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
    public void execute() {
        pivot.pidPower();
    }

    @Override
    public boolean isFinished() {
        return pivot.pidCloseEnough();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            pivot.brake();
            pivot.resetTarget();
        }
    }
}
