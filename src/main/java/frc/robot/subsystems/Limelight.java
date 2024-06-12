package frc.robot.subsystems;

// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;
import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
// import edu.wpi.first.units.Time;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Variables;

import static frc.robot.Constants.GeneralConstants.*;

public class Limelight { // Not technically a subsystem; everything should be static

    private Swerve swerve;

    private static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private static Supplier<Double[]> limelightData = () -> limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, });
    private StructLogEntry<Pose2d> visionPoseLog;
    private StructPublisher<Pose2d> visionPoseNT;
    

    public Limelight(Swerve swerve) {
        super();
        this.swerve = swerve;

        setPipeline(0);
        turnOffLED();
        visionPoseNT = NetworkTableInstance.getDefault().getStructTopic("/Vision/Pose", Pose2d.struct).publish();
        var log = DataLogManager.getLog();
        visionPoseLog = StructLogEntry.create(log, "/Vision/Pose", Pose2d.struct);
    }

    public Pose2d getEstimatedPose() {
      var data = limelightData.get();
      return new Pose2d(data[0], data[1], Rotation2d.fromDegrees(data[5]));
    }

    public void periodic() {
      var data = limelightData.get();
      Translation2d translation = new Translation2d(data[0], data[1]);
      // z, roll, pitch
      Rotation2d yaw = Rotation2d.fromDegrees(data[5]);
      double latency = data[6];
      double tagCount = data[7];
      // double tagSize = data[8];
      // double averageTagDistance = data[9];
      double averageTagArea = data[10];

      Pose2d pose = new Pose2d(translation, yaw);

      Pose2d swervePose = Swerve.getPose();
      ChassisSpeeds speeds = swerve.getRobotRelativeSpeeds();
      double poseDifference = swervePose.getTranslation().getDistance(pose.getTranslation());

      if (translation.getNorm() < 0.1 || tagCount == 0) {
        log("Limelight State", "No Data");
        log("Limelight xyStds", 1000);
        log("Limelight degStds", 1000);
        log("Limelight Latency", latency);

      
        visionPoseLog.append(pose, (long) (Timer.getFPGATimestamp() - latency / 1000));
        visionPoseNT.set(pose);
        return;
      }

      double xyStds, degStds;
      if (tagCount >= 2) {
        // we see 2 tags, so we're super correct
        xyStds = 0.5;
        degStds = 6;
        log("Limelight State", "2 tag");
      } else if (averageTagArea > .4 && poseDifference < 0.5) {
        // we see 1 tag, but it's a big one
        xyStds = 1.0;
        degStds = 12;
        log("Limelight State", "1 big tag");
      } else if (averageTagArea > .05 && poseDifference < .3) {
        // we see 1 tag, but it's a small one and it's close to where we are
        xyStds = 2.0;
        degStds = 30;
        log("Limelight State", "1 small tag");
      } else {
        log("Limelight State", "Bad Data");
        log("Limelight xyStds", 1000);
        log("Limelight degStds", 1000);
        log("Limelight Latency", latency);

      
        visionPoseLog.append(pose, (long) (Timer.getFPGATimestamp() - latency / 1000));
        visionPoseNT.set(pose);
        return;
      }

      double translationSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      xyStds *= (translationSpeed * 0.25 + 1);
      degStds *= (speeds.omegaRadiansPerSecond * 0.5 + 1);

      log("Limelight xyStds", xyStds);
      log("Limelight degStds", degStds);
      log("Limelight Latency", latency);
      
      visionPoseLog.append(pose, (long) (Timer.getFPGATimestamp() - latency / 1000));
      visionPoseNT.set(pose);

      Swerve.poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - latency / 1000, VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
    }


    public static void setSide() {
        if (DriverStation.getAlliance().isPresent()) {
            Variables.isBlueAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
        } else {
            Variables.isBlueAlliance = true;
        }

        if (Variables.isBlueAlliance) {
            setPipeline(0);
        } else {
            setPipeline(1);
        }
    }

    public static void setPipeline(int number) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(number); // which pipeline we use
    }

    public static void turnOffLED() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public void overrideOdometry() {
      var pose = getEstimatedPose();
      if (pose.getTranslation().getNorm() > 0.5) {
        swerve.resetOdometry(pose);
      }
    }
}
