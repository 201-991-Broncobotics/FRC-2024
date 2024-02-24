package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class Limelight extends SubsystemBase {
  LimelightIO io;
  Drive drive;
  LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();
  public Limelight(LimelightIO io, Drive drive) {
    this.io = io;
    this.drive = drive;
  }

  public void periodic() {
    io.updateInputs(inputs);

    if (inputs.pose.getX() == 0) {
      return;
    }

    if (inputs.tagCount > 0) {
      double poseDifference = drive.getPose().getTranslation().getDistance(inputs.pose);
      double xyStds, degStds;

      // copied from limelight docs
      // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
      if (inputs.tagCount >= 2) {
        xyStds = 0.5;
        degStds = 6;
      }
      else if (inputs.averageTagArea > .4 && poseDifference < 0.5) {
        xyStds = 1.0;
        degStds = 12;
      } else if (inputs.averageTagArea > .05 && poseDifference < .5) {
        xyStds = 2.0;
        degStds = 30;
      } else {
        return;
      }

      drive.addVisionMeasurement(xyStds, degStds, new Pose2d(inputs.pose, inputs.yaw), inputs.latency);
    }

  }
}
