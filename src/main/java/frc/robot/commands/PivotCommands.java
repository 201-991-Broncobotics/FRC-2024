package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.DoubleSupplier;

public class PivotCommands {
  private PivotCommands() {}

  public static Command basicOperatorControl(Pivot pivot, DoubleSupplier rotationSupplier) {
    return Commands.run(
        () -> {
          double delta = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.1, 1);

          var target = pivot.getCurrentPosition().plus(Rotation2d.fromDegrees(delta*4));

          pivot.setTargetPosition(target);
        },
        pivot);
  }

  public static Command goToPosition(Pivot pivot, Rotation2d position) {
    return new Command() {
      @Override
      public void initialize() {
        pivot.setTargetPosition(position);
      }

      @Override
      public boolean isFinished() {
        return pivot.getCurrentPosition().minus(position).getDegrees() < 1;
      }
    };
  }

  public static Command goToIntake(Pivot pivot) {
    return goToPosition(pivot, PivotConstants.intakePosition);
  }
}
