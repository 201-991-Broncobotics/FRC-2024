package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIETalon;

import static frc.robot.Constants.TuningConstants.*;
import static frc.robot.Constants.PivotConstants.*;

public class Pivot extends SubsystemBase {
    
    private PIETalon pivot_motor;

    public Pivot() {
        pivot_motor = new PIETalon(pivot_motor_ID, pivot_motor_max_continuous_current, pivot_motor_max_current, 
            pivot_motor_brake, pivot_motor_clockwise_positive, starting_angle, starting_angle, amp_angle, pivot_motor_min_percent_output, 
            pivot_motor_max_percent_output, pivot_motor_max_percent_output_per_second, pivot_motor_gear_ratio, 
            pivot_motor_invert_sensor, pivot_motor_calibration_time, pivot_p, pivot_i, pivot_e
        );
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
        SmartDashboard.putNumber("Pivot Current", pivot_motor.getCurrent());
        SmartDashboard.putNumber("Pivot Position", pivot_motor.getEncoderPosition());
    }
}
