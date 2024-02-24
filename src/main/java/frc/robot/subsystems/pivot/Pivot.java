package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  public Pivot(PivotIO io) {
    this.io = io;
  }

  public void setTargetPosition(Rotation2d target) {
    if (target.getRadians() < PivotConstants.pivotStart.getRadians()) {
      target = PivotConstants.pivotStart;
    }

    if (target.getRadians() > PivotConstants.pivotEnd.getRadians()) {
      target = PivotConstants.pivotEnd;
    }

    // surely this isnt necessary
    if (target.getRadians() < PivotConstants.pivotStart.getRadians()
        || target.getRadians() > PivotConstants.pivotEnd.getRadians()) {
      throw new IllegalArgumentException("Target position is out of bounds");
    }
    io.setTargetPosition(target);
  }

  public Rotation2d getCurrentPosition() {
    return inputs.pivotCurentAngle;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
  }
}
