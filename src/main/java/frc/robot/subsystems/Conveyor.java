package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.EasyCANSparkMax;

import static frc.robot.Constants.ConveyorConstants.*;

public class Conveyor extends SubsystemBase {
    private EasyCANSparkMax conveyor_motor;

    public Conveyor() {
        conveyor_motor = new EasyCANSparkMax(conveyor_motor_id, conveyor_motor_type, conveyor_motor_max_continuous_current, 
            conveyor_motor_max_current, conveyor_motor_invert, conveyor_motor_brake, conveyor_motor_max_percent_output_per_second);
    }

    public void intake() {
        conveyor_motor.set(conveyor_intake_speed);
        SmartDashboard.putString("Conveyor State", "Intaking");
    }

    public void outtake() {
        conveyor_motor.set(conveyor_outtake_speed);
        SmartDashboard.putString("Conveyor State", "Outtaking");

    }

    public void stop() {
        conveyor_motor.set(0);
        SmartDashboard.putString("Conveyor State", "Stopped");
    }

    public double getCurrent() {
        return conveyor_motor.getCurrent();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Conveyor Motor Current", getCurrent());
    }
}
