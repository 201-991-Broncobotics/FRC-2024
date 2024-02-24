package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
  private final CANSparkMax motor;

  public IntakeIOReal() {
    motor = new CANSparkMax(IntakeConstants.intakeCANId, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    inputs.current = motor.getOutputCurrent();
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
