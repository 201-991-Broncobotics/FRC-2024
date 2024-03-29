package frc.robot.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.subsystems.Swerve;

public class Autonomous {
    static String selectedAuto = "Center C3 C2 F1 C1";
    static Command auto = null;
    static boolean lastIsBlueAlliance = Variables.isBlueAlliance;
    public static void disabledPeriodic() {
        String newSelectedAuto = SmartDashboard.getString("Auto Selector String", "Center C3 C2 F1 C1");

        if (!selectedAuto.equals(newSelectedAuto) || lastIsBlueAlliance != Variables.isBlueAlliance) {
            auto = new PathPlannerAuto(newSelectedAuto);
            selectedAuto = newSelectedAuto;
            lastIsBlueAlliance = Variables.isBlueAlliance;
        }
    }

    public static Command getAutonomousCommand(Swerve swerve) {
        return auto;
    }
}
