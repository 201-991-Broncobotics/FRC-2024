package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIGETalon;

import static frc.robot.Constants.TuningConstants.*;
import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.PivotConstants.*;

public class Pivot extends SubsystemBase {
    
    private PIGETalon pivot_motor;
    private DoubleLogEntry targetAngleLog, currentAngleLog;

    public Pivot() {
        pivot_motor = new PIGETalon(pivot_motor_ID, pivot_motor_max_continuous_current, pivot_motor_max_current, 
            pivot_motor_brake, pivot_motor_clockwise_positive, starting_angle, starting_angle, amp_angle, pivot_motor_min_percent_output, 
            pivot_motor_max_percent_output, pivot_motor_max_percent_output_per_second, pivot_motor_gear_ratio, 
            pivot_motor_invert_sensor, pivot_motor_calibration_time, pivot_p, pivot_i, pivot_g, pivot_e, pivot_zero
        );

        var log = DataLogManager.getLog();

        targetAngleLog = new DoubleLogEntry(log, "Pivot/TargetAngle");
        currentAngleLog = new DoubleLogEntry(log, "Pivot/CurrentAngle");
    }

    public void brake() {
        pivot_motor.brake();
    }

    public void move(double power) {
        pivot_motor.power(power);
    }
    
    public void pidPower() {
        pivot_motor.pidPower();
    }

    public void resetTarget() {
        pivot_motor.resetTarget();
    }

    public void setTarget(double angle) {
        pivot_motor.setTarget(angle);
    }

    public double getTargetAngle() {
        return pivot_motor.getTarget();
    }
    
    public double getPosition() {
        return pivot_motor.getEncoderPosition();
    }
    
    public double getCurrentError() {
        return getTargetAngle() - getPosition();
    }

    public boolean pidCloseEnough() {
        return Math.abs(getCurrentError()) < pivot_angle_tolerance;
    }

    public boolean isFree() {
        return pivot_motor.getCurrent() < pivot_motor_free_current;
    }

    @Override
    public void periodic() {
        log("Pivot Current", pivot_motor.getCurrent());
        log("Pivot Motor Get", pivot_motor.get());
        log("Pivot Motor Sine", Math.sin((pivot_zero - pivot_motor.getEncoderPosition()) * Math.PI / 180));
        log("Possibly kG", pivot_motor.get() / Math.sin((pivot_zero - pivot_motor.getEncoderPosition()) * Math.PI / 180));
        log("Current Pivot Angle", pivot_motor.getEncoderPosition());
        log("Target Pivot Angle", pivot_motor.getTarget());
        log("Pivot At Target", pidCloseEnough() ? "yes" : "no");

        targetAngleLog.append(pivot_motor.getTarget());
        currentAngleLog.append(pivot_motor.getEncoderPosition());
    }
}
