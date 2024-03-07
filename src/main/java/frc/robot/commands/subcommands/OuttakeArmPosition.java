package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class OuttakeArmPosition extends Command {
    private Pivot pivot;

    private double target_angle;

    public OuttakeArmPosition(Pivot pivot) {
        this.pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {

        // TODO: Use Limelight to determine target_angle

        target_angle = 40;

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
