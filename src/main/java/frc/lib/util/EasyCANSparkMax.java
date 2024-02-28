package frc.lib.util;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;

public class EasyCANSparkMax {

    private final CANSparkMax motor;

    public EasyCANSparkMax(int CANID, MotorType motorType, double continuousCurrentLimit, double peakCurrentLimit, 
                           boolean clockwise_positive, boolean brake, double max_percent_output_per_second) {
        motor = new CANSparkMax(CANID, motorType);
        
        motor.restoreFactoryDefaults();
        motor.setInverted(clockwise_positive); // uninverted is counterclockwise positive

        Timer.delay(1.0);
        
        motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);

        motor.setOpenLoopRampRate(1.0 / max_percent_output_per_second);
        motor.setClosedLoopRampRate(1.0 / max_percent_output_per_second);

        motor.setSmartCurrentLimit((int) Math.round(continuousCurrentLimit));
        motor.setSecondaryCurrentLimit(peakCurrentLimit);
    }

    public void set(double speed) {
        motor.set(speed);
    }

    public void stop() {
        set(0);
    }

    public double getSpeed() {
        return motor.get();
    }

    public double getCurrent() {
        return motor.getOutputCurrent();
    }
}
