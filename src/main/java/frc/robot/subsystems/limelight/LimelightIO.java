package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {
  @AutoLog
  public static class LimelightIOInputs {
    public double latency = 0.0;

    public Translation2d pose = new Translation2d();
    public Rotation2d roll = new Rotation2d(), pitch = new Rotation2d(), yaw = new Rotation2d();
    public double tagCount = 0.0, tagSpan = 0.0, averageTagDistance = 0.0, averageTagArea = 0.0;
  }

  public default void updateInputs(LimelightIOInputs inputs) {}
}
