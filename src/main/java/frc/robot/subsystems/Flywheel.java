package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIETalon;
import monologue.Logged;

import static frc.robot.Constants.FlywheelConstants.*;

public class Flywheel extends SubsystemBase implements Logged {

    private PIETalon top_flywheel_motor;
    private PIETalon bottom_flywheel_motor;

    public Flywheel() {

        top_flywheel_motor = new PIETalon(top_flywheel_motor_ID, flywheel_motors_max_continuous_current, flywheel_motors_max_current, 
            flywheel_motors_brake, flywheel_motors_clockwise_positive, 0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, flywheel_motors_min_percent_output, 
            flywheel_motors_max_percent_output, flywheel_motors_max_percent_output_per_second, flywheel_motors_gear_ratio, 
            flywheel_motors_invert_sensor, flywheel_motors_calibration_time, 0, 0, 1
        );
        
        bottom_flywheel_motor = new PIETalon(bottom_flywheel_motor_ID, flywheel_motors_max_continuous_current, flywheel_motors_max_current, 
            flywheel_motors_brake, flywheel_motors_clockwise_positive ^ flywheel_motors_opposite, 0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, flywheel_motors_min_percent_output, 
            flywheel_motors_max_percent_output, flywheel_motors_max_percent_output_per_second, flywheel_motors_gear_ratio, 
            flywheel_motors_invert_sensor, flywheel_motors_calibration_time, 0, 0, 1
        ); // invert and opposite: TT -> F, TF -> T, FT -> T, FF -> F; ^ is XOR

        top_flywheel_motor.disableLimiting();
        bottom_flywheel_motor.disableLimiting();
    }

    public void outtake() {
        top_flywheel_motor.power(flywheel_outtake_power);
        bottom_flywheel_motor.power(flywheel_outtake_power);
    }

    public void amp() {
        top_flywheel_motor.power(flywheel_amp_power);
        bottom_flywheel_motor.power(flywheel_amp_power);
    }

    public void stop() {
        top_flywheel_motor.power(0);
        bottom_flywheel_motor.power(0);
    }
    
    public double getAverageCurrent() {
        return (top_flywheel_motor.getCurrent() + bottom_flywheel_motor.getCurrent()) / 2.0;
    }

    public boolean isFree() {
        return getAverageCurrent() < flywheel_motors_free_current;
    }

    @Override
    public void periodic() {
        log("average_current", getAverageCurrent());
        log("top_current", top_flywheel_motor.getCurrent());
        log("bottom_current", bottom_flywheel_motor.getCurrent());
    }
}
