package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Swerve;

public class SysIDRoutineSwerve {
    SysIdRoutine routine;
    
    public SysIDRoutineSwerve(Swerve swerve) {
        // routine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism((volts) -> swerve.driveVoltage(volts), swerve::logSysID));

    }
    public static Command quastitatic(SysIdRoutine.Direction direction) {
        return null;
    }
}
