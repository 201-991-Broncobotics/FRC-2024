package frc.robot.subsystems.limelight;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightIOReal implements LimelightIO {
  NetworkTableEntry botpose;

  public LimelightIOReal() {
    botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue");
  }

  public void updateInputs(LimelightIOInputs inputs) {
    var vals = botpose.getDoubleArray(new double[] {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,});
  }

}
