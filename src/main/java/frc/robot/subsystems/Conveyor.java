package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.EasyCANSparkMax;

import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.ConveyorConstants.*;

public class Conveyor extends SubsystemBase {
    private EasyCANSparkMax conveyor_motor;

    public Conveyor() {
        conveyor_motor = new EasyCANSparkMax(conveyor_motor_id, conveyor_motor_type, conveyor_motor_max_continuous_current, 
            conveyor_motor_max_current, conveyor_motor_clockwise_positive, conveyor_motor_brake, conveyor_motor_max_percent_output_per_second);
    }

    public void intake() {
        conveyor_motor.set(conveyor_intake_speed);
        log("Conveyor State", "Intaking");
    }

    public void outtake() {
        conveyor_motor.set(conveyor_outtake_speed);
        log("Conveyor State", "Outtaking");

    }

    public void stop() {
        conveyor_motor.set(0);
        log("Conveyor State", "Off");
    }

    public double getCurrent() {
        return conveyor_motor.getCurrent();
    }

    public boolean isFree() {
        return getCurrent() < conveyor_motor_free_current;
    }

    @Override
    public void periodic() {
        log("Conveyor Motor Current", getCurrent());
    }
}
