package frc.robot.autonomous;

import static frc.robot.Constants.GeneralConstants.log;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Variables;
import frc.robot.subsystems.Swerve;

public class ShootingMath {
    // all in meters and seconds
    public static final Translation2d redSpeaker = new Translation2d(16.3322, 5.5);
    public static final Translation2d blueSpeaker = new Translation2d(0.2286, 5.5);
    public static final double speaker_height = 2.04311 + 0.0508; // speaker height

    public static double flywheel_rpm_offset = 0; // rpm of flywheel added to calculated value for tuning purposes

    public static final double outtakeForwardOffset = 0.19685; // how far outtake pivot is from center of robot
    public static final double outtakeVerticalOffset = 0.5852089896; // how far up outtake pivot is from center of robot
    
    public static final double z = speaker_height - outtakeVerticalOffset;
    public static final double max_distance = 5; // 6.8?

    // Aidan shooting variables: (sry these should go in constants)
    public static final double a6 = 2.5631 * Math.pow(10.0, -8.0);
    public static final double b6 = 2.3932 * Math.pow(10.0, -10.0);
    public static final double c6 = -1.3862 * Math.pow(10.0, -8.0);
    public static final double d6 = 1.16461;
    public static final double l6 = -1.0202 * Math.pow(10.0, 7.0); 
    public static final double m6 = -4.70122;
    public static final double n6 = -0.000596977;
    public static final double o6 = 10.0282;

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

        double dInches = d * 39.37; // distance in inches

        double angle_radians = Math.toRadians(a6 / (Math.tan(Math.toRadians(b6 * dInches - c6))) + d6);

        Rotation2d angle = Rotation2d.fromRadians(angle_radians);
        log("Auto Aim Pivot Angle (deg)", angle.getDegrees());

        return angle;
    }

    public static double targetShootingRPM() {
        Translation2d speaker = Variables.isBlueAlliance ? blueSpeaker : redSpeaker;

        // base it on the angle we want to be, not the angle we are currently at
        Translation2d pivot = Swerve.getPose().getTranslation().plus(new Translation2d(outtakeForwardOffset, 0).rotateBy(drivetrainAngle()));
        
        double d = speaker.getDistance(pivot);

        if (d > max_distance) {
            d = max_distance;
        }

        log("Auto Aim Estimated Distance", d);

        double dInches = d * 39.37; // distance in inches

        // I'm not gonna bother trying to explain this, just ask me and I will show you the desmos - Aidan Turico
        // also I unfortunately did the math in degrees and inches so I have to convert a lot of variables
        double angle_radians = Math.toRadians(a6 / (Math.tan(b6 * dInches - c6)) + d6); // angle for pitch of outtake
        double Ecx = (11.16 * (Math.cos(angle_radians))) + (3.56 * (Math.sin(angle_radians))) + -7.554629; // coords for the end of the outtake
        double Ecy = (11.16 * (Math.sin(angle_radians))) + (-3.56 * (Math.cos(angle_radians))) + 22.458911; 
        double Tx = l6 * Math.pow(dInches, m6) + n6 * dInches + o6; // target x coord translation
        double V = (dInches - Tx - Ecx) / (Math.cos(angle_radians)) * Math.sqrt(386.088 / (2 * ((Math.tan(angle_radians)) * (dInches - Tx - Ecx) - ((4.77 * Tx) / 18.11) - 78.13 + Ecy))); // velocity of note

        double targetRPM = (60 * V) / (3.5 * Math.PI) + flywheel_rpm_offset; // flywheel velocity 
        log("Auto Aim target shooting RPM", targetRPM);
        log("Auto Aim flywheel RPM offset", flywheel_rpm_offset);

        return targetRPM;
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
