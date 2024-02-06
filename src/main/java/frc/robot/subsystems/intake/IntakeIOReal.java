package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX motor;
  private final StatusSignal<Double> current;

  public IntakeIOReal() {
    motor = new TalonFX(IntakeConstants.intakeCANId);
    current = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(20.0, current);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(current);
    inputs.current = current.getValue();
  }

  @Override
  public void setPower(double power) {
    motor.set(power);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
}
