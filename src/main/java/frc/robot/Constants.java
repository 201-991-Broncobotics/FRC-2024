// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Replaying from a log file. */
    REPLAY
  }

  public static class DriveConstants {
    // Front Left, Front Right, Back Left, Back Right
    // IN DEGREES
    public static final double[] moduleAngles = new double[] {-9.49, -39.99, 50.18, 145.63};

    public static final int[] turnCanIds = new int[] {2, 5, 8, 10};
    public static final int[] driveCanIds = new int[] {1, 4, 7, 11};
    public static final int[] canCoderIds = new int[] {3, 6, 9, 12};
  }
  public static class IntakeConstants {
    public static final int intakeCANId = 20;

    public static final double intakeOnPower = 0.65;
    /** how much time after the note is completely thorugh the intake that it's at the end of the conveyor */
    public static final double intakeToConveyorTime = 0.2;
  }

  public static class PivotConstants {
    public static final int pivotCANId = 13;

    // 5:1 planetary + 5:1 planetary + 48:21 chain
    public static final double pivotGearRatio = 57.1429;

    // the mechanically stopped beginning of the pivot is at a negative angle from the ground
    // 0 degrees is parallel to the ground and facing the same direction as the back of the robot
    public static final Rotation2d pivotStart = Rotation2d.fromRadians(-0.13969988112131737);
    public static final Rotation2d pivotEnd = Rotation2d.fromDegrees(129.056487);
    /** spot where the intake conveyor can take a note from the intake */
    public static final Rotation2d intakePosition = Rotation2d.fromDegrees(60);
    // less than this angle, we won't be able to shoot a note
    public static final Rotation2d minShootingAngle = Rotation2d.fromDegrees(25);

    public static final double p = 0.5, i = 0, d = 0;
  }

  public static class ShooterConstants {
    public static final int conveyorCANId = 19;
    public static final int topShooterCANId = 16;
    public static final int bottomShooterCANId = 17;

    public static final double shootingPower = -0.75;
    public static final double conveyorPower = 0.35;
  }

  public static class HangConstants {
    public static final int leaderCANId = 14;
    public static final int followerCANId = 15;
  }

  public static class TuningConstants {
        /* Swerve Drive Constants */

        public static final double drive_motor_p = 0.05,
                                   drive_static_voltage = 0.32, 
                                   drive_equilibrium_voltage = 1.51, 
                                   drive_acceleration_voltage = 0.27, // SYSID values: KS, KV, KA; they are automatically divided by 12 later
                                   max_linear_speed = 10, // feet per second; theoretical max is 13.5
                                   max_angular_speed = 225; // degrees per second; theoretical max is theoretical maximum is max_linear_speed * 46.6436741705 which is roughly 629.6896013018

        /* PathPlanner PID Constants */

        public static final double autonomous_max_linear_speed = 8,
                                   autonomous_ramp_up_time_linear = 2.5, // in seconds to reach max 
                                   autonomous_max_angular_speed = 180, 
                                   autonomous_ramp_up_time_angular = 1.5,
                                   autonomous_translation_p_controller = 2, 
                                   autonomous_angle_p_controller = 4;
        
        /* Driving PID Constants */

        public static final double teleop_angle_p = 0.165,
                                   teleop_angle_i = 0,
                                   teleop_angle_e = 1.35,
                                   teleop_translation_p = 0.4, 
                                   teleop_translation_i = 0,
                                   teleop_translation_e = 1;
        
        public static final double turn_slow_ratio = 2; // because slowing down rotation and translation by the same factor is insane
                                    // ex. it its 4, then 0.6 translation ratio goes to a 0.9 turning ratio
        
  }

  public static final class TeleopSwerveConstants {
        public static final double swerve_min_pid_rotation = 0.05,
                                   swerve_max_pid_rotation = 0.8, 
                                   swerve_calibration_time = 0.5, 
                                   swerve_min_manual_translation = 0.05, 
                                   swerve_min_manual_rotation = 0.02,

                                   teleop_swerve_slow_factor = 0.5, 
                                   
                                   vision_tolerance = 15, 
                                   
                                   teleop_rotation_percent = 0.75, 
                                   swerve_bumper_turn_sensitivity = 0.35; // ratio of teleop swerve rotation speed vs maximum swerve rotation speed
    }
}
