package frc.robot.autonomous;

import static frc.robot.Constants.GeneralConstants.log;

import edu.wpi.first.math.geometry.*;

import frc.robot.subsystems.Swerve;
import frc.robot.Variables;

public class ShootingMath {
    // all in meters and seconds
    public static final Translation2d redSpeaker = new Translation2d(16.3322, 5.55);
    public static final Translation2d blueSpeaker = new Translation2d(0.2286, 5.55);
    public static final double speaker_height = 2.04311; // speaker height

    public static final double g = 9.81;
    public static final double v = 13.56663838; // exit velocity, in m/s

    public static final double outtakeForwardOffset = 0.19685; // how far outtake pivot is from center of robot
    public static final double outtakeVerticalOffset = 0.5852089896; // how far up outtake pivot is from center of robot
    
    public static final double z = speaker_height - outtakeVerticalOffset;
    public static final double max_distance = 5; // 6.8?

    // assuming no robot velocity; everything else is negligible 
    public static Rotation2d drivetrainAngle() {
        var speaker = Variables.isBlueAlliance ? blueSpeaker : redSpeaker;

        // vector from our current robot pose to the speaker; works no matter what side we're on
        Translation2d vector = speaker.minus(Swerve.getPose().getTranslation());

        // add 180 bc this tells us the angle the outtake is supposed to face, and outtake is at 180° yaw
        Rotation2d angle = vector.getAngle().plus(Rotation2d.fromDegrees(180));
        
        log("Auto Aim Drivetrain Angle (deg)", angle.getDegrees());

        return angle;
    }

    public static Rotation2d pivotAngle() {
        Translation2d speaker = Variables.isBlueAlliance ? blueSpeaker : redSpeaker;

        // base it on the angle we want to be, not the angle we are currently at
        Translation2d pivot = Swerve.getPose().getTranslation().plus(new Translation2d(outtakeForwardOffset, 0).rotateBy(drivetrainAngle()));
        
        double d = speaker.getDistance(pivot);

        if (d > max_distance) {
            d = max_distance; // lol
        }

        double angle_radians = Math.atan(
            (v * v * d - Math.sqrt(v * v * v * v * d * d - g * d * d * (g * d * d + 2 * v * v * z))) / (g * d * d)
        );

        Rotation2d angle = Rotation2d.fromRadians(angle_radians);
        log("Auto Aim Pivot Angle (deg)", angle.getDegrees());

        return angle;
    }
    
    /* COORDINATES (ALL IN METERS)
     * x = 0.2286, or 16.3322 (still 0.2286 for red)
     * y = 5.55, regardless of size
     * field width = 16.5608
     * field height = 8.02
     * 
     * remember, 0 points straight out, and positive should be counterclockwise. CHECK THIS!!!
     */

    /* Limelight stats (not for here)
     * 
     * Forward offset from drivetrain center: -0.2201853864
     * Z offset from drivetrain center: 0.3957396454
     * Yaw: 180°
     * Pitch: 23°
     * 
     * Zero yaw = towards intake
     * Zero angle = towards red alliance
     */
}
