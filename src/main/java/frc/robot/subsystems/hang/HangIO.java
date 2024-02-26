package frc.robot.subsystems.hang;

import org.littletonrobotics.junction.AutoLog;

public interface HangIO {
  @AutoLog
  public static class HangIOInputs {
    public double mainCurrent = 0.0;
    public double followerCurrent = 0.0;
  }

  default void updateInputs(HangIOInputs inputs) {}

  default void setSpeed(double power) {}
}
