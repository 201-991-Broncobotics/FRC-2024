package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.GeneralConstants.*;
import com.ctre.phoenix6.StatusSignal;
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
    
    StatusSignal<Double> topCurrent;
    StatusSignal<Double> bottomCurrent;

    StatusSignal<Double> topVoltage;
    StatusSignal<Double> bottomVoltage;

    StatusSignal<Double> topRPS;
    StatusSignal<Double> bottomRPS;

    public Flywheel() {
        
        top = new TalonFX(top_flywheel_motor_ID);
        bottom = new TalonFX(bottom_flywheel_motor_ID);
        
        var config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        var slot0 = new Slot0Configs();
        // add 0.05V for static friction
        slot0.kS = .05;
        // each rps = 0.01 more volts
        slot0.kV = 0.01;
        // each rps of error = 0.02 more volts
        slot0.kP = 0.02;

        top.getConfigurator().apply(config);
        bottom.getConfigurator().apply(config);

        topCurrent = top.getSupplyCurrent();
        bottomCurrent = bottom.getSupplyCurrent();
        topVoltage = top.getMotorVoltage();
        bottomVoltage = bottom.getMotorVoltage();
        topRPS = top.getVelocity();
        bottomRPS = bottom.getVelocity();
        
        StatusSignal.setUpdateFrequencyForAll(50, topRPS, bottomRPS, topCurrent, bottomCurrent, topVoltage, bottomVoltage);
        
        top.optimizeBusUtilization();
        bottom.optimizeBusUtilization();
    }

    public void outtake() {
        targetRPS = flywheel_shooting_rps;
        log("Flywheel State", "Outtaking");
    }

    public void amp() {
        targetRPS = flywheel_amp_rps;
        log("Flywheel State", "Amping");
    }

    public void stop() {
        targetRPS = 0;
        log("Flywheel State", "Off");
    }
    
    public double getAverageCurrent() {
        return (topCurrent.getValueAsDouble() + bottomCurrent.getValueAsDouble()) / 2.0;
    }

    public boolean isFree() {
        return getAverageCurrent() < flywheel_motors_free_current;
    }
    
    public boolean isAtSpeed() {
      double tolerance = targetRPS == flywheel_shooting_rps ? 100 : 50;
      double topError = Math.abs(targetRPS - topRPS.getValueAsDouble());
      double bottomError = Math.abs(targetRPS - bottomRPS.getValueAsDouble());

      return topError < tolerance && bottomError < tolerance;
    

    }

    @Override
    public void periodic() {
        log("Flywheel Motor Average Current", getAverageCurrent());
        log("Flywheel Top Current", topCurrent.getValueAsDouble());
        log("Flywheel Bottom Current", bottomCurrent.getValueAsDouble());
        log("Flywheel Top Voltage", topVoltage.getValueAsDouble());
        log("Flywheel Bottom Voltage", bottomVoltage.getValueAsDouble());
        log("Flywheel Top RPS", topRPS.getValueAsDouble());
        log("Flywheel Bottom RPS", bottomRPS.getValueAsDouble());

        top.setControl(request.withVelocity(targetRPS));
        bottom.setControl(request.withVelocity(targetRPS));
    }
}
