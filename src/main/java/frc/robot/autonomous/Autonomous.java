package frc.robot.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.subsystems.Swerve;

public class Autonomous {

    public static Command getAutonomousCommand(Swerve swerve) {
        String selectedAuto = SmartDashboard.getString("Auto Selector String", "Center C3 C2 F1 C1");
        SmartDashboard.putString("Autonomous", (Variables.isBlueAlliance ? "Blue" : "Red") + " " + selectedAuto);
        
        return new PathPlannerAuto(selectedAuto);
    }
}
