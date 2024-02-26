package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class PIECalculator {
    private final double kP, kI, kE, minPower, maxPower;

    private double previous_time, integral, correction, targetPosition;

    public PIECalculator(double kP, double kI, double kE, double minPower, double maxPower, double startingPosition) {
        this.kP = kP;
        this.kI = kI;
        this.kE = kE;
        this.minPower = minPower;
        this.maxPower = maxPower;
        resetTarget(startingPosition);
    }

    public void resetTarget(double targetPosition) { // resets integral
        this.targetPosition = targetPosition;
        integral = 0;
        previous_time = Timer.getFPGATimestamp();
    }

    public void forceResetTarget(double targetPosition) { // doesn't reset integral
        this.targetPosition = targetPosition;
        previous_time = Timer.getFPGATimestamp();
    }

    public double update(double currentPosition) {

        double error = targetPosition - currentPosition;
        double delta_time = Timer.getFPGATimestamp() - previous_time;
        previous_time = Timer.getFPGATimestamp();

        double position_correction, integral_correction;

        position_correction = getPECorrection(error, kP, kE, minPower, maxPower);
        
        integral += error * delta_time;
        integral_correction = integral * kI;

        if (Math.abs(position_correction + integral_correction) < minPower) return 0;

        return Math.max(-maxPower, Math.min(maxPower, position_correction + integral_correction));
    }

    public static final double getPECorrection(double error, double proportional, double exponent, double min_power, double max_power) {
        if (Math.abs(error) == 0) return 0;

        double multiplier = max_power;
        if (error < 0) {
            error = 0 - error;
            multiplier = -max_power;
        }

        error *= proportional / max_power; // as a proportion of maximum power
        error = Math.min(error, 1);

        double calculated_power = Math.pow(error, exponent) * multiplier;

        return (Math.abs(calculated_power) < min_power) ? 0 : calculated_power;
    }

    public double getCorrection() {
        return correction;
    }

    public double getTarget() {
        return targetPosition;
    }
}