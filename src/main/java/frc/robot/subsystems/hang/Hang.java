package frc.robot.subsystems.hang;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
  HangIO io;
  HangIOInputsAutoLogged inputs = new HangIOInputsAutoLogged();

  public Hang(HangIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hang", inputs);
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public Command operatorControl(DoubleSupplier speedSupplier) {
    return Commands.run(() -> {
      double speed = speedSupplier.getAsDouble();
      setSpeed(speed * 0.1);
    }, this);
  }
}
