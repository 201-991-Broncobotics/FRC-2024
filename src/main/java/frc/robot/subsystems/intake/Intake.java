package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public final IntakeIO io;
  public final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void turnOn() {
    io.setPower(IntakeConstants.intakeOnPower);
  }

  public void turnOff() {
    io.setPower(0.0);
  }

  public Command on() {
    return new InstantCommand(this::turnOn, this);
  }

  public Command off() {
    return new InstantCommand(this::turnOff, this);
  }
}
