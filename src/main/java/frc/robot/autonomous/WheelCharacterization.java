package frc.robot.autonomous;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BaseFalconSwerveConstants;
import frc.robot.subsystems.Swerve;
import static frc.robot.Constants.GeneralConstants.*;

public class WheelCharacterization extends CommandBase {
  private Swerve swerve;

  DoubleSupplier gyroYawRads;

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double[] startWheelAngles = new double[4];

  // meters
  private double driveRadius = Math.hypot(BaseFalconSwerveConstants.trackWidth / 2, BaseFalconSwerveConstants.wheelBase / 2);

  // meters
  double currentEffectiveWheelRadius = 0.0;

  public WheelCharacterization(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    this.gyroYawRads = () -> swerve.getGyroYaw().getRadians();
  }

  @Override
  public void initialize() {
    lastGyroYawRads = gyroYawRads.getAsDouble();
    accumGyroYawRads = 0.0;

    startWheelAngles = swerve.getWheelRadiusCharacterizationPosition();
  }

  @Override
  public void execute() {
    swerve.drive(new Translation2d(0, 0), Math.PI / 2, true, true);

    accumGyroYawRads += MathUtil.angleModulus(gyroYawRads.getAsDouble() - lastGyroYawRads);

    lastGyroYawRads = gyroYawRads.getAsDouble();
    double averageWheelPosition = 0.0;
    double[] wheelPositiions = swerve.getWheelRadiusCharacterizationPosition();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelAngles[i]);
    }
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;

    log("Current Effective Wheel Radius (m)", currentEffectiveWheelRadius);
  }

  @Override
  public void end(boolean interrupted) {
    if (accumGyroYawRads < 10) {
      System.out.println("Wheel Characterization Failed: Not enough data collected");
    } else {
      System.out.println("Effective Wheel Raidus inches: " + Units.metersToInches(currentEffectiveWheelRadius));
    }

  }
}
