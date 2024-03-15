package frc.robot.autonomous;

import static frc.robot.Constants.GeneralConstants.log;

import edu.wpi.first.math.geometry.*;

import frc.robot.subsystems.Swerve;
import frc.robot.Variables;

public class ShootingMath {

    // all in meters and seconds
    public static final Translation2d redSpeaker = new Translation2d(16.2686, 5.55);
    public static final Translation2d blueSpeaker = new Translation2d(0.2922, 5.55);
    public static final double speaker_height = 2.04311;

    public static final double outtakeVerticalOffset = 0.5852089896; // how far up outtake pivot is from center of robot
    
    public static final double g = 9.81;
    public static final double v = 13.56663838; // exit velocity, in m/s
    
    public static final double z = speaker_height - outtakeVerticalOffset;

    public static final double outtakeForwardOffset = 0.19685; // how far outtake pivot is from center of robot
    public static final double max_distance = 5; // 6.8?

    public static final int num_recursions = 3;
        
    /* COORDINATES (ALL IN METERS)
        * x = 0.2922, or 16.2686 (still 0.2286 for red)
        * y = 5.55, regardless of size
        * field width = 16.5608
        * field height = 8.02
    
        Limelight stats (not for here)
        * 
        * Forward offset from drivetrain center: -0.2201853864
        * Z offset from drivetrain center: 0.3957396454
        * Yaw: 180°
        * Pitch: 23°s
        * 
        * Zero yaw = towards intake
        * Zero angle = towards red alliance
        */

    // assuming no robot velocity; everything else is negligible 
    public static Rotation2d drivetrainAngle() {
        double drivetrainAngle = calculateRecursiveAngles(Swerve.getPose().getTranslation(), Variables.isBlueAlliance ? blueSpeaker : redSpeaker, Swerve.velocity)[0];

        log("Auto Aim Drivetrain Angle (deg)", drivetrainAngle);

        return Rotation2d.fromDegrees(drivetrainAngle);
    }

    public static Rotation2d pivotAngle() {
        double pivotAngle = calculateRecursiveAngles(Swerve.getPose().getTranslation(), Variables.isBlueAlliance ? blueSpeaker : redSpeaker, Swerve.velocity)[1];

        log("Auto Aim Pivot Angle (deg)", pivotAngle);

        return Rotation2d.fromDegrees(pivotAngle);
    }

    /** This function does not account for robot velocity; for here, we are assuming the pivot is "forward"
     * @return a double array consisting of { Swerve angle, pivot angle, time } in degrees and seconds */ 
    public static double[] calculateBasicAngles(Translation2d current_position, Translation2d target_position, double height_offset, double exit_velocity, double gravity, double forward_offset, boolean checkDistance) {
        
        Translation2d vector = target_position.minus(current_position);
        Translation2d pivot = current_position.plus(new Translation2d(-forward_offset, 0).rotateBy(vector.getAngle())); // negative because we consider pivot as positive in this frame of reference

        Translation2d new_vector = target_position.minus(pivot);

        double d = new_vector.getNorm(); // distance

        if (d > max_distance && checkDistance) {
            throw new IllegalArgumentException();
        }

        double z = height_offset;
        double v = exit_velocity;
        double g = gravity;

        double a = g * d * d;
        double b_ = v * v * d; // -b / 2
        double c = g * d * d + 2 * v * v * z;

        double pivot_angle = Math.atan((b_ - Math.sqrt(b_ * b_ - a * c)) / a);

        return new double[] {
            vector.getAngle().getDegrees(), 
            pivot_angle * 180.0 / Math.PI, 
            d / v / Math.cos(pivot_angle)
        };
    }

    /** This function does account for robot velocity, but is dependent on simple angles
     * @return a double array consisting of { Swerve angle, pivot angle, time } in degrees and seconds */ 
    public static double[] calculateAdvancedAngles(Translation2d current_position, Translation2d target_position, double height_offset, double exit_velocity, double gravity, double forward_offset, boolean checkDistance, double[] previous_values, Translation2d robot_velocity) {

        Translation2d adjustedVelocity = robot_velocity.rotateBy(Rotation2d.fromDegrees(0 - previous_values[0]));

        double tv = adjustedVelocity.getY(); // tangential velocity; positive means we need to angle more to right
        double nv = adjustedVelocity.getX(); // positive is towards

        Translation2d pivot = current_position.plus(new Translation2d(-forward_offset, 0).rotateBy(Rotation2d.fromDegrees(previous_values[0]))); // negative because we consider pivot as positive in this frame of reference
        Translation2d new_vector = target_position.minus(pivot);

        double d = new_vector.getNorm() - nv * previous_values[2]; // adjusted distance
        
        if (d > max_distance && checkDistance) {
            throw new IllegalArgumentException();
        }

        double z = height_offset;
        double v = exit_velocity;
        double g = gravity;

        double a = g * d * d;
        double b_ = v * v * d; // -b / 2
        double c = g * d * d + 2 * v * v * z;

        double pivot_angle = Math.atan((b_ - Math.sqrt(b_ * b_ - a * c)) / a);

        double tangential_addition = Math.atan(-tv * previous_values[2] / d) * 180.0 / Math.PI;

        return new double[] {
            previous_values[0] + tangential_addition, 
            pivot_angle * Math.PI / 180.0, 
            d / v / Math.cos(pivot_angle)
        };
    }

    /** This function does account for robot velocity, but is dependent on simple angles
     * @return a double array consisting of { Swerve angle, pivot angle } in degrees */ 
    public static double[] calculateRecursiveAngles(Translation2d current_position, Translation2d target_position, Translation2d robot_velocity) {
        try {
            double[] angles = calculateBasicAngles(current_position, target_position, z, v, g, outtakeForwardOffset, num_recursions == 0);

            for (int i = 0; i < num_recursions; i++) {
                angles = calculateAdvancedAngles(current_position, target_position, z, v, g, outtakeForwardOffset, i + 1 == num_recursions, angles, robot_velocity);
            }

            return new double[] {
                angles[0] + 180, 
                angles[1]
            };
        } catch (Exception e) {
            return new double[] {
                target_position.minus(current_position).getAngle().getDegrees() + 180, 
                25
            };
        }
    }
    
}
