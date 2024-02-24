package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;

  public Shooter(ShooterIO io) {
    this.io = io;
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

  public Command shootersOnCommand() {
    return new InstantCommand(this::shootersOn, this);
  }

  public Command shootersOffCommand() {
    return new InstantCommand(this::shootersOff, this);
  }

  public Command conveyorOnCommand() {
    return new InstantCommand(this::conveyorOn, this);
  }

  public Command conveyorOffCommand() {
    return new InstantCommand(this::conveyorOff, this);
  }
}
