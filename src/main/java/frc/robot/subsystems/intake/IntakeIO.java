package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double current = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPower(double power) {}

  public default void setVoltage(double voltage) {}
}
