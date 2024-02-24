package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class ShooterIOReal implements ShooterIO {
  private TalonFX topShooter;
  private TalonFX bottomShooter;
  private CANSparkMax conveyor;

  public ShooterIOReal() {
    topShooter = new TalonFX(topShooterCANId);
    bottomShooter = new TalonFX(bottomShooterCANId);
    conveyor = new CANSparkMax(conveyorCANId, MotorType.kBrushless);

    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 25;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    topShooter.getConfigurator().apply(config);

    // todo: test
    // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    bottomShooter.getConfigurator().apply(config);

    conveyor.restoreFactoryDefaults();
    conveyor.setIdleMode(IdleMode.kBrake);
    conveyor.setSmartCurrentLimit(20);
  }

  public void setTopShooterPower(double power) {
    topShooter.set(power);
  }

  public void setBottomShooterPower(double power) {
    bottomShooter.set(power);
  }

  public void setConveyorPower(double power) {
    conveyor.set(power);
  }
}
