// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;


  private static final ModuleConfigs configs = new ModuleConfigs();

  public ModuleIOTalonFX(int index) {
    if (index >= 4) {
      throw new RuntimeException("Invalid module index");
    }
    driveTalon = new TalonFX(DriveConstants.driveCanIds[index]);
    turnTalon = new TalonFX(DriveConstants.turnCanIds[index]);
    cancoder = new CANcoder(DriveConstants.canCoderIds[index]);

    Timer.delay(.5);

    driveTalon.getConfigurator().apply(configs.swerveDriveFXConfig);
    turnTalon.getConfigurator().apply(configs.swerveAngleFXConfig);
    cancoder.getConfigurator().apply(configs.swerveCANcoderConfig);

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent, drivePosition, turnPosition);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();

    setDrivePosition(0.0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);


    inputs.drivePosition = drivePosition.getValueAsDouble();
    inputs.driveVelocity = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrent = driveCurrent.getValueAsDouble();
    inputs.cancoderPosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocity = turnVelocity.getValueAsDouble();
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrent = turnCurrent.getValueAsDouble();
  }

  @Override
  public void setTurnPosition(double pos) {
    turnTalon.setPosition(pos);
  }

  @Override
  public void setDrivePosition(double pos) {
    driveTalon.setPosition(pos);
  }

  @Override
  public void setTurnControl(PositionVoltage p) {
    turnTalon.setControl(p);
  }

  @Override
  public void setDriveControl(VelocityVoltage v) {
    driveTalon.setControl(v);
  }

  @Override
  public void setDriveControl(DutyCycleOut d) {
    driveTalon.setControl(d);
  }
}
