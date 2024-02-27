package frc.robot.commands.defaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hang;

import static frc.robot.Constants.GeneralConstants.*;

public class TeleopHang extends Command { // lol

    private Hang hang;

    private DoubleSupplier motorSup;

    public TeleopHang(Hang hang, DoubleSupplier motorSup) {
        this.hang = hang;
        addRequirements(hang);

        this.motorSup = motorSup;
    }

    @Override
    public void execute() {
        // Account for Stick Drift
        double motorVal = signedPower(motorSup.getAsDouble());

        // Move Arm
        hang.move(
            motorVal * 0.2
        );
    }

}
