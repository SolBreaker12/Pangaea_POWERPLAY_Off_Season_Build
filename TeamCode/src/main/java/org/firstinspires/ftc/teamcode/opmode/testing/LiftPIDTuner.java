package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class LiftPIDTuner extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private LiftSubsystem lift;



    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;

        waitForStart();

        while (opModeIsActive()) {

            lift.read();
            lift.loop();
            lift.write();
        }

    }
}
