package frc.robot.subsystems.hang;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.HangConstants;

public class HangIOReal implements HangIO {

  TalonFX leader;
  TalonFX follower; 

  StatusSignal<Double> leaderCurrent;
  StatusSignal<Double> followerCurrent;
  StatusSignal<Double> leaderPosition;
  StatusSignal<Double> followerPosition;

  public HangIOReal() {

    leader = new TalonFX(HangConstants.leaderCANId);
    follower = new TalonFX(HangConstants.followerCANId);

    // TODO: should this boolean be false?? do they rotate in the same ways??
    follower.setControl(new Follower(HangConstants.leaderCANId, true));

    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    leaderCurrent = leader.getTorqueCurrent();
    followerCurrent = follower.getTorqueCurrent();
    leaderPosition = leader.getPosition();
    followerPosition = follower.getPosition();

    // don't really need this to update that often b/c it's just for data visualization
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, leaderCurrent, followerCurrent, followerPosition, leaderPosition);
    leader.optimizeBusUtilization(1.0);
    follower.optimizeBusUtilization(1.0);

  }

  public void updateInputs(HangIOInputs inputs) {
    inputs.mainCurrent = leaderCurrent.getValueAsDouble();
    inputs.followerCurrent = followerCurrent.getValueAsDouble();
  }

  public void setSpeed(double power) {
    leader.set(power);
  }
  

}
