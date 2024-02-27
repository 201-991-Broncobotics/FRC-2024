package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public void on() {
    io.setPower(IntakeConstants.intakeOnPower);
    SmartDashboard.putString("intake", "on");
  }

  public void off() {
    io.setPower(0.0);
    SmartDashboard.putString("intake", "off");
  }

  public Command onCommand() {
    return new InstantCommand(this::on, this);
  }

  public Command offCommand() {
    return new InstantCommand(this::off, this);
  }

  public double getCurrent() {
    return inputs.current;
  }
}
