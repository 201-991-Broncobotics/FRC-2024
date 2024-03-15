package frc.robot.commands.defaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.autonomous.ShootingMath;
import frc.robot.subsystems.Pivot;

import static frc.robot.Constants.GeneralConstants.*;

public class TeleopPivot extends Command {

    private Pivot pivot;

    private DoubleSupplier motorSup;

    private boolean lastBypass = false;

    public TeleopPivot(Pivot pivot, DoubleSupplier motorSup) {
        this.pivot = pivot;
        addRequirements(pivot);

        this.motorSup = motorSup;
    }

    @Override
    public void execute() {
        // Account for Stick Drift
        double motorVal = signedPower(motorSup.getAsDouble());

        // Move Arm
        if (Variables.bypass_angling) {
            pivot.setTarget(ShootingMath.pivotAngle().getDegrees());
            pivot.pidPower();
            lastBypass = true;
        } else {
            if (lastBypass) {
                pivot.resetTarget();
            }
            pivot.move(motorVal * 0.4);
            lastBypass = false;
        }
    }
}
