package frc.robot;

public class Variables {
    public static double slowdown_factor = 1.0; // multiplicative; i.e. 0.5 -> go at 50% speed

    public static boolean in_auto = false;
    public static boolean in_teleop = false;

    public static boolean isBlueAlliance = true;

    public static boolean invert_rotation = false;
    public static boolean bypass_angling = false;

    public static double x;
    public static double y;
    public static double angle;

    public static boolean bypass_rotation = false;

    /* COORDINATES OF SPEAKER
     * x = 0.2286, or 16.3322 (still 0.2286 for red)
     * y = 5.55, or 8.02 - 5.55 for red
     * width = 16.5608
     * 
     * remember, 0 points straight out, and positive should be counterclockwise. CHECK THIS!!!
     */
}
