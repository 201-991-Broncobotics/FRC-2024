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
}
