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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.Conversions;
import frc.robot.Constants.DriveConstants;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Module {
  ModuleIO io;
  ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  Rotation2d angleOffset;

  int moduleNumber;

      /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(BaseFalconSwerveConstants.driveKS, BaseFalconSwerveConstants.driveKV, BaseFalconSwerveConstants.driveKA);


  public Module(ModuleIO io, int moduleNumber) {
    this.io = io;
    this.angleOffset = Rotation2d.fromDegrees(DriveConstants.moduleAngles[moduleNumber] + 90);
    this.moduleNumber = moduleNumber;


    resetToAbsolute();

    io.setDrivePosition(0.0);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        io.setTurnControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
  }


  public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.RPSToMPS(inputs.driveVelocity, BaseFalconSwerveConstants.wheelCircumference), 
      // todo: this should be turn position but its not working??
            inputs.cancoderPosition
        );
    }
  
  public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(
          Conversions.rotationsToMeters(inputs.drivePosition, BaseFalconSwerveConstants.wheelCircumference), 
      // todo: this should be turn position but its not working??
          inputs.cancoderPosition
      );
  }

  public double[] getCurrents() {
    return new double[] {
        inputs.turnCurrent, // stator instead of torque because we
        inputs.driveCurrent  // only care about absolute value
    };
}
private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
  if (isOpenLoop) {
      driveDutyCycle.Output = desiredState.speedMetersPerSecond / BaseFalconSwerveConstants.maxSpeed;
      io.setDriveControl(driveDutyCycle);
  } else {
      driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, BaseFalconSwerveConstants.wheelCircumference);
      driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
      io.setDriveControl(driveVelocity);
  }
}

  public Rotation2d getCANCoder() {
    return inputs.cancoderPosition;
  }

  public void resetToAbsolute() {
    double absolutePosition = inputs.cancoderPosition.getRotations() - angleOffset.getRotations();
    io.setTurnPosition(absolutePosition);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(moduleNumber), inputs);
  }
}
