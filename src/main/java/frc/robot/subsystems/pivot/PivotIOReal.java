package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.PivotConstants;

public class PivotIOReal implements PivotIO {
  private final TalonFX pivotMotor;
  // in rotations!!
  private final StatusSignal<Double> position;
  // in rotations!!
  double zeroPosition;
  Rotation2d targetPosition;
  TalonFXConfiguration config;
  PositionDutyCycle request;

  private final StatusSignal<Double> current;
  private final StatusSignal<Double> voltage;

  public PivotIOReal() {
    pivotMotor = new TalonFX(PivotConstants.pivotCANId);

    config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Slot0 =
        new Slot0Configs()
            .withKP(PivotConstants.p)
            .withKI(PivotConstants.i)
            .withKD(PivotConstants.d);

    pivotMotor.getConfigurator().apply(config);

    // in rotations!!
    position = pivotMotor.getPosition();
    current = pivotMotor.getTorqueCurrent();
    voltage = pivotMotor.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, position, current, voltage);
    pivotMotor.optimizeBusUtilization(1.0);

    request = new PositionDutyCycle(position.getValueAsDouble()).withSlot(0);

    // PivotConstants.pivotStart
    zeroPosition = position.getValueAsDouble();
    targetPosition = angleFromEncoder(zeroPosition);
  }

  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, current,voltage);
    double currentPosition = position.getValueAsDouble();

    inputs.pivotCurentAngle = angleFromEncoder(currentPosition);
    inputs.pivotTargetAngle = targetPosition;
    inputs.motorCurrent = current.getValueAsDouble();
    inputs.motorVoltage = voltage.getValueAsDouble();
  }

  /** sets the target position where 0 is facing towards the back of the robot */
  public void setTargetPosition(Rotation2d target) {
    targetPosition = target;
    pivotMotor.setControl(request.withPosition(encoderFromAngle(target)));
  }

  private Rotation2d angleFromEncoder(double encoderTicks) {
    // 2048 ticks per rotation
    var delta =
        Rotation2d.fromRotations((encoderTicks - zeroPosition) / PivotConstants.pivotGearRatio);

    return delta.plus(PivotConstants.pivotStart);
  }

  private double encoderFromAngle(Rotation2d angle) {
    var delta =
        angle.minus(PivotConstants.pivotStart).getRotations() * PivotConstants.pivotGearRatio;

    return delta + zeroPosition;
  }
}
