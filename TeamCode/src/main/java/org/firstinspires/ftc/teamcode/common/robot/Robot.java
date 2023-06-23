package org.firstinspires.ftc.teamcode.common.robot;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    public static Robot instance = null;
    public boolean enabled;

    Motor leftBack;
    Motor leftFront;
    Motor rightBack;
    Motor rightFront;

    public static Robot getInstance() {
        if (instance == null)
            instance = new Robot();
        instance.enabled = true;
        return instance;
    }

    public void InitRobot(HardwareMap hardwareMap) {
        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
    }
}
