package frc.robot.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;

public class HuskyLens {
    public final HuskyLens huskylens_camera;

    huskylens_camera = hardwareMap.get(Huskylens.class, "HuskyLens");
    Camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    
}
