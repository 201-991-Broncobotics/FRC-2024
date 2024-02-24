package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightIOReal implements LimelightIO {
  NetworkTableEntry botpose;

  public LimelightIOReal() {
    botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue");
  }

  public void updateInputs(LimelightIOInputs inputs) {
    var vals =
        botpose.getDoubleArray(
            new double[] {
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            });

    // ignore z value
    inputs.pose = new Translation2d(vals[0], vals[1]);
    inputs.roll = Rotation2d.fromDegrees(vals[3]);
    inputs.pitch = Rotation2d.fromDegrees( vals[4]);
    inputs.yaw = Rotation2d.fromDegrees( vals[5]);
    inputs.latency = vals[6];
    inputs.tagCount = vals[7];
    inputs.tagSpan = vals[8];
    inputs.averageTagDistance = vals[9];
    inputs.averageTagArea = vals[10];
  }
}
