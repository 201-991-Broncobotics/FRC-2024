package frc.robot.autonomous;

import static frc.robot.Constants.GeneralConstants.log;

import edu.wpi.first.math.geometry.*;

import frc.robot.subsystems.Swerve;
import frc.robot.Variables;

public class ShootingMath {

    public static final Translation2d redSpeaker = new Translation2d(16.3322, 5.55);
    public static final Translation2d blueSpeaker = new Translation2d(0.2286, 5.55);
    public static final double height = 2.04311; // speaker height

    // TODO: tune
    public static final double outtakeOffset = 0.381; // how far outtake pivot is from center of robot
    public static final double outtakeUp = 0.01; // how far up outtake pivot is from center of robot

    // TODO: offset robot pose b/c we dont shoot from center of robot
    // assuming no robot velocity; everything else is negligible 
    public static Rotation2d drivetrainAngle() {
        var speaker = Variables.isBlueAlliance ? blueSpeaker : redSpeaker;

        // vector from our current robot pose to the speaker
        // this works no matter what side we're on
        var vector = speaker.minus(Swerve.getPose().getTranslation());

        // get the ccw+ angle above the x axis for this vector.
        // for example, if we're blue and shooting into the speaker from right in front of it,
        // this will return 180 because away = 0
        var angle = vector.getAngle().plus(Rotation2d.fromDegrees(180));
        log("Auto Aim Drivetrain Angle (deg)", angle.getDegrees());
        return angle;
    }

    public static Rotation2d pivotAngle() {

        var speaker = Variables.isBlueAlliance ? blueSpeaker : redSpeaker;

        // now, we want to make a triangle with the distance to the speaker as the base,
        // and the height of the center of the speaker as the height
        // then, the angle that we're looking for will be asin(z / distance)
        var distance = speaker.getDistance(Swerve.getPose().getTranslation());
        var z = height - outtakeOffset;

        var angle = Rotation2d.fromRadians(Math.asin(z / distance));

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
     * Forward offset from drivetrain center: -8.668716 * 0.0254
     * Z offset from drivetrain center: 15.580301 * 0.0254
     * Yaw: 180
     * Pitch: 21
     * 
     * It is facing towards the outtake
     * 
     */
}
