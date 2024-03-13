package frc.lib.util;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;

public class PIGETalon {

    private final TalonFX motor;
    private final PIGECalculator pigeCalculator;

    private final DoubleSupplier positionSup;
    private final double calibrationTime, maxPercentOutputPerSecond, multiplier;
    private double minPosition, maxPosition, previousTime, time, lmtPosition, prevPower;

    /** Assumptions: 360 is a full rotation; zeroPosition is the reading at the unstable equilibrium */
    public PIGETalon(int CANID, double continuousCurrentLimit, double peakCurrentLimit, boolean brake, boolean clockwise_positive, 
        double startingAngle, double minPosition, double maxPosition, double minPercentOutput, double maxPercentOutput, 
        double maxPercentOutputPerSecond, double gear_ratio, boolean invertEncoder, double calibrationTime, double kP, double kI, 
        double kG, double kE, double zeroPosition
    ) {

        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.MotorOutput.Inverted = clockwise_positive ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        configuration.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.SupplyCurrentLimit = peakCurrentLimit;
        configuration.CurrentLimits.SupplyCurrentThreshold = continuousCurrentLimit;

        configuration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1.0 / maxPercentOutputPerSecond;
        configuration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1.0 / maxPercentOutputPerSecond;

        configuration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 1.0 / maxPercentOutputPerSecond;
        configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0 / maxPercentOutputPerSecond;

        motor = new TalonFX(CANID);

        motor.getConfigurator().apply(new TalonFXConfiguration());
        motor.getConfigurator().apply(configuration);
        
        Timer.delay(1.0);
        
        multiplier = (invertEncoder ? -1 : 1) * gear_ratio / 360.0; // 2048 per revolution and its in degrees
        motor.setPosition(startingAngle * multiplier);

        this.minPosition = minPosition;
        this.maxPosition = maxPosition;

        this.calibrationTime = calibrationTime;
        this.maxPercentOutputPerSecond = maxPercentOutputPerSecond;

        positionSup = () -> motor.getPosition().getValueAsDouble() / multiplier;

        pigeCalculator = new PIGECalculator(kP, kI, kG, kE, minPercentOutput, maxPercentOutput, startingAngle, zeroPosition);
        time = Timer.getFPGATimestamp();
        previousTime = -1000;
    }

    public void power(double power) { // power = between 0 and 1
        double deltaTime = Timer.getFPGATimestamp() - time;
        time = Timer.getFPGATimestamp();

        double currentPosition = positionSup.getAsDouble();
        
        if (currentPosition < minPosition || lmtPosition <= minPosition) {
            previousTime = -1000;
            power = Math.max(0, power);
            prevPower = power;
            motor.set(0);
            pigeCalculator.resetTarget(minPosition);
            lmtPosition = minPosition;
        } else if (currentPosition > maxPosition || lmtPosition >= maxPosition) {
            previousTime = -1000;
            power = Math.min(0, power);
            prevPower = power;
            motor.set(0);
            pigeCalculator.resetTarget(maxPosition);
            lmtPosition = maxPosition;
        }

        if (power != 0) {
            pigeCalculator.resetTarget(currentPosition);
            lmtPosition = currentPosition;
            previousTime = time;
        } else if (time - previousTime < calibrationTime) {
            pigeCalculator.resetTarget(currentPosition);
        } else {
            power = pigeCalculator.update(currentPosition);
        }

        power = Math.max(prevPower - maxPercentOutputPerSecond * deltaTime, Math.min(prevPower + maxPercentOutputPerSecond * deltaTime, power));
        prevPower = power;

        motor.set(power);
    }

    public void resetSensorPosition(double angle) {
        motor.setPosition(angle * multiplier);
        previousTime = -1000;
        pigeCalculator.resetTarget(angle);
        prevPower = 0;
        motor.set(0);
        lmtPosition = angle;
    }

    public void resetTarget() {
        previousTime = -1000;
        pigeCalculator.resetTarget(positionSup.getAsDouble());
        prevPower = 0;
        motor.set(0);
        lmtPosition = positionSup.getAsDouble();
    }

    public void setTarget(double targetPosition) {
        previousTime = -1000;
        pigeCalculator.resetTarget(targetPosition);
        prevPower = 0;
        motor.set(0);
        lmtPosition = targetPosition;
    }

    public void pidPower() {
        double deltaTime = Timer.getFPGATimestamp() - time;
        time = Timer.getFPGATimestamp();
        lmtPosition = 0; // between min and max
        double power = pigeCalculator.update(positionSup.getAsDouble());
        power = Math.max(prevPower - maxPercentOutputPerSecond * deltaTime, Math.min(prevPower + maxPercentOutputPerSecond * deltaTime, power));
        prevPower = power;

        motor.set(power);
    }

    public void pidPower(double multiplier) {
        double deltaTime = Timer.getFPGATimestamp() - time;
        time = Timer.getFPGATimestamp();
        lmtPosition = 0; // between min and max
        double power = pigeCalculator.update(positionSup.getAsDouble()) * multiplier;
        power = Math.max(prevPower - maxPercentOutputPerSecond * deltaTime, Math.min(prevPower + maxPercentOutputPerSecond * deltaTime, power));
        prevPower = power;
        if (power < -0.95) power = -0.95;
        if (power > 0.95) power = 0.95;

        motor.set(power);
    }

    public void brake() {
        pigeCalculator.resetTarget(positionSup.getAsDouble());
        previousTime = Timer.getFPGATimestamp();
        prevPower = 0;
        motor.set(0);
    }

    public double getTarget() {
        return pigeCalculator.getTarget();
    }

    public void disableLimiting() {
        minPosition = Double.NEGATIVE_INFINITY;
        maxPosition = Double.POSITIVE_INFINITY;
    }

    public double getCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public double getEncoderPosition() {
        return positionSup.getAsDouble();
    }
}