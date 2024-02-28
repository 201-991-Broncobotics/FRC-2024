package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIETalon;

import static frc.robot.Constants.TuningConstants.*;
import static frc.robot.Constants.HangConstants.*;

public class Hang extends SubsystemBase {

    private PIETalon left_hang_motor;
    private PIETalon right_hang_motor;

    public Hang() {

        left_hang_motor = new PIETalon(left_hang_motor_ID, hang_motors_max_continuous_current, hang_motors_max_current, 
            hang_motors_brake, hang_motors_clockwise_positive, 0, -hanging_position, hanging_position, hang_motors_min_percent_output, 
            hang_motors_max_percent_output, hang_motors_max_percent_output_per_second, hang_motors_gear_ratio, 
            hang_motors_invert_sensor, hang_motors_calibration_time, hang_p, hang_i, hang_e
        );
        
        right_hang_motor = new PIETalon(right_hang_motor_ID, hang_motors_max_continuous_current, hang_motors_max_current, 
            hang_motors_brake, hang_motors_clockwise_positive ^ hang_motors_opposite, 0, -hanging_position, hanging_position, hang_motors_min_percent_output, 
            hang_motors_max_percent_output, hang_motors_max_percent_output_per_second, hang_motors_gear_ratio, 
            hang_motors_invert_sensor, hang_motors_calibration_time, hang_p, hang_i, hang_e
        ); // invert and opposite: TT -> F, TF -> T, FT -> T, FF -> F; ^ is XOR

        left_hang_motor.disableLimiting();
        right_hang_motor.disableLimiting();

    }

    public void move(double power) {
        left_hang_motor.power(power);
        right_hang_motor.power(power);
    }

    public void relax() {
        left_hang_motor.setTarget(0);
        right_hang_motor.setTarget(0);
        SmartDashboard.putString("Hang State", "Relaxed");
    }

    public void hang() {
        left_hang_motor.setTarget(hanging_position);
        right_hang_motor.setTarget(hanging_position);
        SmartDashboard.putString("Hang State", "Hanging");
    }

    public boolean isFree() {
        return left_hang_motor.getCurrent() + right_hang_motor.getCurrent() < 2 * hang_motors_free_current;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Hang Motor Position", left_hang_motor.getEncoderPosition());
        SmartDashboard.putNumber("Right Hang Motor Position", right_hang_motor.getEncoderPosition());
        SmartDashboard.putNumber("Average Hang Motor Current", (left_hang_motor.getCurrent() + right_hang_motor.getCurrent()) / 2.0);
    }
}
