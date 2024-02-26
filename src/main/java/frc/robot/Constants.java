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
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public static class DriveConstants {
    // Front Left, Front Right, Back Left, Back Right
    // IN RADIANS
    public static final double[] moduleAngles = new double[] {-2.46, -0.79, 2.45, -1.38};

    public static final int[] turnCanIds = new int[] {2, 5, 8, 11};
    public static final int[] driveCanIds = new int[] {1, 4, 7, 10};
    public static final int[] canCoderIds = new int[] {3, 6, 9, 12};
  }

  public static class IntakeConstants {
    public static final int intakeCANId = 20;

    public static final double intakeOnPower = 0.65;
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
    public static final Rotation2d intakePosition = Rotation2d.fromDegrees(25);
    // less than this angle, we won't be able to shoot a note
    public static final Rotation2d minShootingAngle = Rotation2d.fromDegrees(25);

    public static final double p = 0.00005, i = 0, d = 0;
  }

  public static class ShooterConstants {
    public static final int conveyorCANId = 19;
    public static final int topShooterCANId = 16;
    public static final int bottomShooterCANId = 17;

    public static final double shootingPower = -0.75;
    public static final double conveyorPower = 0.6;
  }

  public static class HangConstants {
    public static final int leaderCANId = 14;
    public static final int followerCANId = 15;
  }
}
