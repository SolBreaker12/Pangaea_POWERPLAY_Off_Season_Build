package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.apriltag.AprilTag;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.globals.Globals;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


public class RobotHardware {

    public SampleMecanumDrive drive;

    public Motor leftBack;
    public Motor leftFront;
    public Motor rightBack;
    public Motor rightFront;

    public Motor.Encoder leftBackEncoder;
    public Motor.Encoder leftFrontEncoder;
    public Motor.Encoder rightBackEncoder;
    public Motor.Encoder rightFrontEncoder;

    public Motor liftMotor;

    public Motor.Encoder liftMotorEncoder;

    public ServoEx servo1;
    public ServoEx servo2;

    public BNO055IMU IMU;

    public AprilTag aprilTag;

    private static RobotHardware instance = null;

    public boolean enabled;

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        if (Globals.USING_IMU) {
            IMU = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            IMU.initialize(parameters);
        }

        drive = new SampleMecanumDrive(hardwareMap);

        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        leftBack.setInverted(true);
        leftFront.setInverted(true);

        leftBack.resetEncoder();
        leftFront.resetEncoder();
        rightBack.resetEncoder();
        rightFront.resetEncoder();

        liftMotor = new Motor(hardwareMap, "ArmMotor1", Motor.GoBILDA.RPM_435);
        liftMotor.setInverted(true);

        servo1 = new SimpleServo(hardwareMap, "Servo1", Globals.CLAW_MIN, Globals.CLAW_MAX, AngleUnit.RADIANS);
        servo2 = new SimpleServo(hardwareMap, "Servo2", Globals.CLAW_MIN, Globals.CLAW_MAX, AngleUnit.RADIANS);

        if (Globals.AUTO) {
            aprilTag = new AprilTag();
        }
    }

    public void loop(LiftSubsystem lift) {
        lift.loop();
    }

    public boolean isNotDriving() {
        return !drive.isBusy();
    }

}
