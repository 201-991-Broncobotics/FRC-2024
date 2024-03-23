package frc.robot;

public class Variables {
    public static double slowdown_factor = 1.0; // multiplicative; i.e. 0.5 -> go at 50% speed

    public static boolean in_auto = false;
    public static boolean in_teleop = false;

    public static boolean isBlueAlliance = true;

    public static boolean invert_rotation = false;
    /** auto aim pivot */
    public static boolean bypass_angling = false;
    /** auto aim drivetrain */
    public static boolean bypass_rotation = false;
}
