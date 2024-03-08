package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Variables;

import static frc.robot.Constants.GeneralConstants.*;

public class Limelight { // Not technically a subsystem; everything should be static

    private static String botpose_key = "botpose_wpi"; // can be botpose, botpose_wpiblue or botpose_wpired;

    private static DoubleSupplier tv = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(-1); // 0 --> nothing, 1 --> something

    private static DoubleSupplier tid = () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDoubleArray(new double[0])[0];
    
    private static DoubleSupplier[] position = new DoubleSupplier[] {
        () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry(botpose_key + Variables.side).getDoubleArray(new double[6])[0], // x 
        () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry(botpose_key + Variables.side).getDoubleArray(new double[6])[1], // y
        () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry(botpose_key + Variables.side).getDoubleArray(new double[6])[5]  // yaw
    }; // Where the robot is, relative to april tag

    private static DoubleSupplier latency = () -> (
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0) + 
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl").getDouble(0)
    ) / 1000.0; // in seconds

    public static void setSide(String newSide) {
        if (newSide.toLowerCase().equals("red")) {
            Variables.side = "red";
        } else if (newSide.toLowerCase().equals("blue")) {
            Variables.side = "blue";
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

    public static Pose2d adjustPoseForSide(Pose2d pose) {
        if (Variables.side.equals("blue")) {
            return pose;
        }
        return new Pose2d(
            pose.getX(), 
            8.02 - pose.getY(), 
            Rotation2d.fromDegrees(-pose.getRotation().getDegrees())
        );
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