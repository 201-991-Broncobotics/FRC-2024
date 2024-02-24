package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.DoubleSupplier;

public class PivotCommands {
  private PivotCommands() {}

  public static Command basicOperatorControl(Pivot pivot, DoubleSupplier rotationSupplier) {
    return Commands.run(
        () -> {
          var target =
              pivot
                  .getCurrentPosition()
                  .plus(Rotation2d.fromDegrees(rotationSupplier.getAsDouble()));

          pivot.setTargetPosition(target);
        },
        pivot);
  }
}
