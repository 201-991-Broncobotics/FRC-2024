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

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(0);
  private final StatusSignal<Double> yaw = pigeon.getYaw();

  public GyroIOPigeon2(boolean phoenixDrive) {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.setYaw(0.0);
    yaw.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization(1.0);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
  }

  @Override
  public void setYaw(double yaw) {
    pigeon.setYaw(yaw);
  }
}
