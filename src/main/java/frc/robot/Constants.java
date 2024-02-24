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
  public static final Mode currentMode = Mode.SIM;

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
    // IN DEGREES
    public static final double[] moduleAngles = new double[] {0, 0, 0, 0};

    public static final int[] turnCanIds = new int[] {1, 4, 7, 10};
    public static final int[] driveCanIds = new int[] {0, 3, 6, 9};
    public static final int[] canCoderIds = new int[] {2, 5, 8, 11};
  }

  public static class IntakeConstants {
    public static final int intakeCANId = 12;

    public static final double intakeOnPower = 0.1;
  }

  public static class PivotConstants {
    public static final int pivotCANId = 13;

    // 80 falcon rotations = 1 pivot rotation
    // 5:1 planetary + 5:1 planetary + 5:1 planetary + 48:15 chain
    public static final int pivotGearRatio = 400;

    // the mechanically stopped beginning of the pivot is at a negative angle from the ground
    // 0 degrees is parallel to the ground and facing the same direction as the back of the robot
    public static Rotation2d pivotStart = Rotation2d.fromDegrees(-8.265688);
    public static Rotation2d pivotEnd = Rotation2d.fromDegrees(129.056487);
    // less than this angle, we won't be able to shoot a note
    public static Rotation2d minShootingAngle = Rotation2d.fromDegrees(25);

    public static final double p = 0.05, i = 0, d = 0;
  }

  public static class ShooterConstants {
    public static final int conveyorCANId = 14;
    public static final int topShooterCANId = 15;
    public static final int bottomShooterCANId = 16;

    public static final double shootingPower = 0.5;
    public static final double conveyorPower = 0.2;
  }
}
