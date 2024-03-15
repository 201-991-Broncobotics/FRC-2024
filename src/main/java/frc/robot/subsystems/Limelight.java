package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Variables;

import static frc.robot.Constants.GeneralConstants.*;

public class Limelight { // Not technically a subsystem; everything should be static
    private static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private static String botpose_key = "botpose_wpiblue"; // can be botpose, botpose_wpiblue or botpose_wpired;

    private static DoubleSupplier tv = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(-1); // 0 --> nothing, 1 --> something

    private static DoubleSupplier tid = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDoubleArray(new double[0])[0];
    
    // private static Supplier<Double[]> limelightData = () -> limelightTable.getEntry(botpose_key).getDoubleArray(new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, });

    private static DoubleSupplier[] position = new DoubleSupplier[] {
        () -> limelightTable.getEntry(botpose_key).getDoubleArray(new double[6])[0], // x 
        () -> limelightTable.getEntry(botpose_key).getDoubleArray(new double[6])[1], // y
        () -> limelightTable.getEntry(botpose_key).getDoubleArray(new double[6])[5]  // yaw
    }; // Where the robot is, relative to april tag

    private static DoubleSupplier latency = () -> (
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0) + 
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl").getDouble(0)
    ) / 1000.0; // in seconds

    /* private static double getX() {
        return limelightData.get()[0];
    } */
    

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

    public static void init() {
        setPipeline(0);
        turnOffLED();
    }

    public static double[] getData() {
        if (tv.getAsDouble() == 0) return new double[] {-12, -12, -12, -12};
        return new double[] {
            tid.getAsDouble(), 
            position[0].getAsDouble(), 
            position[1].getAsDouble(), 
            position[2].getAsDouble()
        };
    }

    public static Pose2d getRobotPosition() {

        log("Limelight Ping", "" + Math.round(
            latency.getAsDouble() * 1000
        ) + " ms");

        return new Pose2d(position[0].getAsDouble(), position[1].getAsDouble(), Rotation2d.fromDegrees(position[2].getAsDouble())); 
    }

    public static double getLatency() {
        return latency.getAsDouble(); 
    }

    public static void setPipeline(int number) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(number); // which pipeline we use
    }

    public static void turnOffLED() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }
    
    public static void toggleLED() {
        if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getDouble(0) != 1) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
        } else {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(4 - NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getDouble(3));
        }
    }
}

/**if (inputs.tagCount > 0) {
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

      var pose = new Pose2d(inputs.pose, inputs.yaw);
      Logger.recordOutput("Limelight/Pose", pose);
      poseEstimator.addVisionMeasurement(pose, inputs.latency, VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));


    } */