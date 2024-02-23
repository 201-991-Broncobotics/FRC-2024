package frc.robot.subsystems.outtake.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {}

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setTopShooterPower(double power) {}

  default void setBottomShooterPower(double power) {}

  default void setConveyorPower(double power) {}
}
