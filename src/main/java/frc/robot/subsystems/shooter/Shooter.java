package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void shootersOn() {
    io.setTopShooterPower(shootingPower);
    io.setBottomShooterPower(shootingPower);
  }

  public void shootersOff() {
    io.setTopShooterPower(0);
    io.setBottomShooterPower(0);
  }

  public void conveyorOn() {
    io.setConveyorPower(conveyorPower);
  }

  public void conveyorOff() {
    io.setConveyorPower(0);
  }
}
