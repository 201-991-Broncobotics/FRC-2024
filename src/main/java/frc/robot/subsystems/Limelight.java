package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Variables;

import static frc.robot.Constants.GeneralConstants.*;

public class Limelight { // Not technically a subsystem; everything should be static

    private static DoubleSupplier latency = () -> (
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0) + 
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl").getDouble(0)
    ) / 1000.0; // in seconds

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

    public static Pose2d getVisionEstimate() {
        double[] values = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });

        return new Pose2d(values[0], values[1], Rotation2d.fromDegrees(values[5]));
    }

    public static void updateSDPE() { // Swerve.poseEstimator

        log("Limelight Ping", "" + Math.round(latency.getAsDouble() * 1000) + " ms");

        double[] values = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });

        Pose2d vision_estimate = new Pose2d(values[0], values[1], Rotation2d.fromDegrees(values[5]));

        double number_of_targets = values[7];
        double average_target_area = values[10];

        if (number_of_targets < 1) {
            log("Vision Pose", "No vision estimate");
            log("Vision Heading", "No vision estimate");
            return;
        }

        log("Vision Pose", "(" + Math.round(vision_estimate.getTranslation().getX() * 100) / 100.0 + ", " + Math.round(vision_estimate.getTranslation().getY() * 100) / 100.0 + ")");
        log("Vision Heading", "" + Math.round(vision_estimate.getRotation().getDegrees() * 100) / 100.0 + " degrees");
    
        // distance from current pose to vision estimated pose
        double poseDifference = Swerve.poseEstimator.getEstimatedPosition().getTranslation().getDistance(vision_estimate.getTranslation());

        double xyStds, degStds;

        if (number_of_targets > 1) { // multiple targets
            xyStds = 0.5;
            degStds = 6;
        } else if (average_target_area > 0.8 && poseDifference < 0.5) { // large area and close to estimated pose
            xyStds = 1.0;
            degStds = 12;
        } else if (average_target_area > 0.1 && poseDifference < 0.3) { // 1 target farther away and estimated pose is close
            xyStds = 2.0;
            degStds = 30;
        } else { // conditions don't match to add a vision measurement
            log("Vision Pose", "Vision estimate did not make sense");
            log("Vision Heading", "Vision estimate did not make sense");
            return;
        }

        log("Vision Pose", "(" + Math.round(vision_estimate.getTranslation().getX() * 100) / 100.0 + ", " + Math.round(vision_estimate.getTranslation().getY() * 100) / 100.0 + ")");
        log("Vision Heading", "" + Math.round(vision_estimate.getRotation().getDegrees() * 100) / 100.0 + " degrees");

        Swerve.poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, degStds * Math.PI / 180.0));
        Swerve.poseEstimator.addVisionMeasurement(vision_estimate, Timer.getFPGATimestamp() - latency.getAsDouble());
    }
}

/*

    Pose2d vision_estimate = Limelight.getRobotPosition();

    if (vision_estimate.getTranslation().getNorm() > 0.1 && (Math.abs(normalizeAngle(getGyroYaw().getDegrees() - vision_estimate.getRotation().getDegrees())) < vision_tolerance)) {
        poseEstimator.addVisionMeasurement(vision_estimate, Timer.getFPGATimestamp() - Limelight.getLatency());
        log("Vision Pose", "(" + Math.round(vision_estimate.getTranslation().getX() * 100) / 100.0 + ", " + Math.round(vision_estimate.getTranslation().getY() * 100) / 100.0 + ")");
        log("Vision Heading", "" + Math.round(vision_estimate.getRotation().getDegrees() * 100) / 100.0 + " degrees");
    } else if (vision_estimate.getTranslation().getNorm() > 0.1) {
        log("Vision Pose", "Vision estimate did not make sense");
        log("Vision Heading", "Vision estimate did not make sense");
    } else {
        log("Vision Pose", "No vision estimate");
        log("Vision Heading", "No vision estimate");
    }

 */


/*

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

        var pose = new Pose2d(inputs.pose, inputs.yaw);
        Logger.recordOutput("Limelight/Pose", pose);
        poseEstimator.addVisionMeasurement(pose, inputs.latency, VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
    } 

*/