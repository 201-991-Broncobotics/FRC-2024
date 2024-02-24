package frc.robot.subsystems.limelight;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Translation3d;

public interface LimelightIO {
  @AutoLog
  public static class LimelightIOInputs {
    public double latency = 0.0;

    public Translation3d pose = new Translation3d(); 
    public double roll = 0.0, pitch = 0.0, yaw = 0.0;
    public double tagCount = 0.0, tagSpan = 0.0, averageTagDistance = 0.0, averageTagArea = 0.0;
  }

  public default void updateInputs(LimelightIOInputs inputs) {
  }
}
