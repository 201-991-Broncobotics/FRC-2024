package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.EasyCANSparkMax;
import monologue.Logged;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase implements Logged {

    private EasyCANSparkMax intake_motor;

    public Intake() {
        intake_motor = new EasyCANSparkMax(intake_motor_id, intake_motor_type, intake_motor_max_continuous_current, 
            intake_motor_max_current, intake_motor_clockwise_positive, intake_motor_brake, intake_motor_max_percent_output_per_second);
    }

    public void intake() {
        intake_motor.set(intake_motor_speed);
        SmartDashboard.putString("Intake State", "Intaking");
    }

    public void stop() {
        intake_motor.set(0);
        SmartDashboard.putString("Intake State", "Stopped");
    }

    public double getCurrent() {
        return intake_motor.getCurrent();
    }

    public boolean isFree() {
        return getCurrent() < intake_motor_free_current;
    }

    @Override
    public void periodic() {
        log("current", getCurrent());
    }
}
