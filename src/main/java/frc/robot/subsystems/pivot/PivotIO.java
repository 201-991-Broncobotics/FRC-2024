package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public Rotation2d pivotCurentAngle = new Rotation2d();
    public Rotation2d pivotTargetAngle = new Rotation2d();
    public double motorCurrent = 0.0;
    public double motorVoltage = 0.0;
  }

  default void updateInputs(PivotIOInputs inputs) {}

  default void setTargetPosition(Rotation2d target) {}
}
