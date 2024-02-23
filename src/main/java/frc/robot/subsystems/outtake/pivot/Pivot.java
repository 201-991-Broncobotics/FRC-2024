package frc.robot.subsystems.outtake.pivot; 

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  private final PivotIO io;

  public Pivot(PivotIO io) {
    this.io = io;
  }

  public void setTargetPosition(Rotation2d target) {
    if (target.getRadians() < PivotConstants.pivotStart.getRadians() 
    || target.getRadians() > PivotConstants.pivotEnd.getRadians()) {
      throw new IllegalArgumentException("Target position is out of bounds");
    }
    io.setTargetPosition(target);
  }


}
