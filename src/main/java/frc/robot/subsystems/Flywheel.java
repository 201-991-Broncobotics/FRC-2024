package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.GeneralConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static frc.robot.Constants.FlywheelConstants.*;

public class Flywheel extends SubsystemBase {

    TalonFX top;
    TalonFX bottom;

    VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    double targetRPS = 0;

    DoubleSupplier topCurrent;
    DoubleSupplier bottomCurrent;

    DoubleSupplier topVoltage;
    DoubleSupplier bottomVoltage;

    DoubleSupplier topRPS;
    DoubleSupplier bottomRPS;

    public Flywheel() {

        top = new TalonFX(top_flywheel_motor_ID);
        bottom = new TalonFX(bottom_flywheel_motor_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        var slot0 = new Slot0Configs();
        // add this much for static friction
        slot0.kS = .05;
        // each rps = this many more volts
        slot0.kV = 0.12;
        // each rps of error = this many more volts
        slot0.kP = 0.13;

        config.Slot0 = slot0;

        top.getConfigurator().apply(config);
        bottom.getConfigurator().apply(config);

        topCurrent = () -> top.getSupplyCurrent().getValueAsDouble();
        bottomCurrent = () -> bottom.getSupplyCurrent().getValueAsDouble();
        topVoltage = () -> top.getMotorVoltage().getValueAsDouble();
        bottomVoltage = () -> bottom.getMotorVoltage().getValueAsDouble();
        topRPS = () -> top.getVelocity().getValueAsDouble();
        bottomRPS = () -> bottom.getVelocity().getValueAsDouble();

    }

    public void outtake() {
        targetRPS = flywheel_shooting_rpm / 60;
        log("Flywheel State", "Outtaking");
    }

    public void amp() {
        targetRPS = flywheel_amp_rpm / 60;
        log("Flywheel State", "Amping");
    }

    public void stop() {
        targetRPS = 0;
        log("Flywheel State", "Off");
    }

    // add backpressure while intaking to make sure we dont send the note too far forwards
    public void intake() {
      targetRPS = flywheel_intake_rpm / 60;
      log("Flywheel State", "Intaking");
    }

    public double getAverageCurrent() {
        return (topCurrent.getAsDouble() + bottomCurrent.getAsDouble()) / 2.0;
    }

    public boolean isFree() {
        return getAverageCurrent() < flywheel_motors_free_current;
    }

    public boolean isAtSpeed() {
        double tolerance = Math.sqrt(flywheel_shooting_rpm / 60);
        double topError = Math.abs(targetRPS - topRPS.getAsDouble());
        double bottomError = Math.abs(targetRPS - bottomRPS.getAsDouble());

        return topError < tolerance && bottomError < tolerance;
    }

    @Override
    public void periodic() {
        log("Flywheel Motor Average Current", getAverageCurrent());
        log("Flywheel Top Current", topCurrent.getAsDouble());
        log("Flywheel Bottom Current", bottomCurrent.getAsDouble());
        log("Flywheel Top Voltage", topVoltage.getAsDouble());
        log("Flywheel Bottom Voltage", bottomVoltage.getAsDouble());
        log("Flywheel Top RPS", topRPS.getAsDouble());
        log("Flywheel Bottom RPS", bottomRPS.getAsDouble());
        log("Flywheel Target RPS", targetRPS);
        log("Flywheel At Speed", isAtSpeed() ? "yes" : "no");

        top.setControl(request.withVelocity(targetRPS));
        bottom.setControl(request.withVelocity(targetRPS));
    }
}
