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

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePosition = 0.0;
    public double driveVelocity = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrent = 0.0;

    public Rotation2d cancoderPosition = new Rotation2d();

    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocity = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrent = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setDrivePosition(double pos) {}

  public default void setTurnPosition(double pos) {}

  public default void setTurnControl(PositionVoltage p) {}

  public default void setDriveControl(DutyCycleOut d) {}

  public default void setDriveControl(VelocityVoltage v) {}

}
