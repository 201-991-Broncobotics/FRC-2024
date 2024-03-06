package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import monologue.Logged;

public class Limelight extends SubsystemBase implements Logged {
    NetworkTableEntry limelight;
    private double latency = 0.0;

    private Translation3d translation = new Translation3d();
    private Rotation3d rotation = new Rotation3d();
    private Pose2d pose = new Pose2d();
    private Rotation2d roll = new Rotation2d(), pitch = new Rotation2d(), yaw = new Rotation2d();
    private double tagCount = 0.0, tagSpan = 0.0, averageTagDistance = 0.0, averageTagArea = 0.0;

    private Swerve swerve;
    
  
    public Limelight(Swerve swerve) {
        limelight = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue");
        // setPipeline(0);
        // turnOffLED();
      this.swerve = swerve;
    }

  @Override
  public void periodic() {
    var vals =
        limelight.getDoubleArray(new double[] {});

    if (vals.length == 0) {
      log("status", "disconnected");
      return;
    }

    translation = new Translation3d(vals[0], vals[1], vals[2]);
    roll = Rotation2d.fromDegrees(vals[3]);
    pitch = Rotation2d.fromDegrees(vals[4]);
    yaw = Rotation2d.fromDegrees(vals[5]);
    latency = vals[6];
    tagCount = vals[7];
    tagSpan = vals[8];
    averageTagDistance = vals[9];
    averageTagArea = vals[10];

    pose = new Pose2d(vals[0], vals[1], yaw);
    rotation = new Rotation3d(roll.getRadians(), pitch.getRadians(), yaw.getRadians());

    log("translation", translation);
    log("yaw", yaw);
    log("pose", pose);
    log("latency", latency);
    log("tagCount", tagCount);
    log("rotation", rotation);
    log("averageTagArea", averageTagArea);

    if (translation.getNorm() < 0.1 || tagCount == 0) {
        log("status", "no_data");
    }

    if (yaw.minus(swerve.getGyroYaw()).getDegrees() > 10) {
      log("status", "not_aligned");
    }


    double poseDifference = swerve.getPose().getTranslation().getDistance(pose.getTranslation());
    double xyStds, degStds; 

    if (tagCount >= 2) {
      xyStds = 0.5;
      degStds = 6;
    } else if (averageTagArea > 0.4 && poseDifference < 0.5) {
      xyStds = 1;
      degStds = 12;
    } else if (averageTagArea > 0.05 && poseDifference < 0.5) {
      xyStds = 2;
      degStds = 30;
    } else {
      log("status", "bad_data");
      return;
    }

    log("status", "good_data");
    log("xyStds", xyStds);
    log("degStds", degStds);

    swerve.addVisionMeasurement(xyStds, degStds, pose, latency);
  }

    public double getLatency() {
        return latency;
    }

    // public static void setPipeline(int number) {
    //     NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    //     NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(number); // which pipeline we use
    // }
    //
    // public static void turnOffLED() {
    //     NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    // }
}
