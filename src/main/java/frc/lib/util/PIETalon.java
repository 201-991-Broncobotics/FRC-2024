package frc.lib.util;

public class PIETalon extends PIGETalon {
    public PIETalon(int CANID, double continuousCurrentLimit, double peakCurrentLimit, boolean brake, boolean clockwise_positive, 
        double startingAngle, double minPosition, double maxPosition, double minPercentOutput, double maxPercentOutput, 
        double maxPercentOutputPerSecond, double gear_ratio, boolean invertEncoder, double calibrationTime, double kP, double kI, double kE
    ) {
        super(CANID, continuousCurrentLimit, peakCurrentLimit, brake, clockwise_positive, startingAngle, 
        minPosition, maxPosition, minPercentOutput, maxPercentOutput, maxPercentOutputPerSecond, 
        gear_ratio, invertEncoder, calibrationTime, kP, kI, 0, kE, startingAngle);
    }
}