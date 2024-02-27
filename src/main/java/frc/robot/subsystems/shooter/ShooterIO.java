package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double topShooterCurrent = 0.0;
    public double bottomShooterCurrent = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setTopShooterPower(double power) {}

  default void setBottomShooterPower(double power) {}

  default void setConveyorPower(double power) {}
}
