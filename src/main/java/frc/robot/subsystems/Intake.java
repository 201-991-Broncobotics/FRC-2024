package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.EasyCANSparkMax;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {

    private EasyCANSparkMax intake_motor;

    public Intake() {
        intake_motor = new EasyCANSparkMax(intake_motor_id, intake_motor_type, intake_motor_max_continuous_current, 
            intake_motor_max_current, intake_motor_invert, intake_motor_brake, intake_motor_max_percent_output_per_second);
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Motor Current", getCurrent());
    }
}