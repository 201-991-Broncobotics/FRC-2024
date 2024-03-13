package frc.lib.util;

public class PIECalculator extends PIGECalculator {

    public PIECalculator(double kP, double kI, double kE, double minPower, double maxPower, double startingPosition) {
        super(kP, kI, 0, kE, minPower, maxPower, startingPosition, startingPosition);
    }

}