package frc.robot.commands.defaultCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hang;

import static frc.robot.Constants.GeneralConstants.*;

public class TeleopHang extends Command { // lol

    private Hang hang;

    private DoubleSupplier motorSup;

    private BooleanSupplier leftSup, rightSup;

    public TeleopHang(Hang hang, DoubleSupplier motorSup, BooleanSupplier leftSup, BooleanSupplier rightSup) {
        this.hang = hang;
        addRequirements(hang);

        this.motorSup = motorSup;

        this.leftSup = leftSup;
        this.rightSup = rightSup;
    }

    @Override
    public void execute() {
        // Account for Stick Drift
        double motorVal = signedPower(motorSup.getAsDouble());

        // Move Arm
        hang.moveLeft(motorVal * 0.4 + (leftSup.getAsBoolean() ? 0.4 : 0));
        hang.moveRight(motorVal * 0.4 + (rightSup.getAsBoolean() ? 0.4 : 0));
    }

}
