package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.apriltag.AprilTag;
import org.firstinspires.ftc.teamcode.common.commandbase.base.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.globals.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Autonomous
public class Left_1 extends LinearOpMode {


    private final RobotHardware robot = RobotHardware.getInstance();
    private LiftSubsystem lift;
    private DrivetrainSubsystem drivetrain;

    private double avgLoopTime;

    private double loopTime;
    private int totalLoops;
    private double totalLoopTime;

    private AprilTag tagDetection;

    @Override
    public void runOpMode() throws InterruptedException {


        CommandScheduler.getInstance().reset();
        Globals.SIDE = Globals.Side.LEFT;
        Globals.AUTO = true;
        Globals.USING_IMU = false;

        robot.init(hardwareMap, telemetry);
        lift = new LiftSubsystem(robot);
        drivetrain = new DrivetrainSubsystem(robot.drive, true);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.enabled = true;

        tagDetection = new AprilTag();
        robot.aprilTag.Init(hardwareMap);

        while (!isStarted()) {
            robot.aprilTag.Run(telemetry);
        }

        robot.aprilTag.AssignPark();

        tagDetection.finalPark = robot.aprilTag.finalPark;


        drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        Pose2d firstInter = new Pose2d(49.25, 0, Math.toRadians(-45));
        Pose2d firstDeposit = new Pose2d(54.25, -5, 0);
        Pose2d secondInter = new Pose2d(49.25, 0, Math.toRadians(135));
        Pose2d thirdInter = new Pose2d(26.25, 0 , 0);

        Pose2d[] parking = new Pose2d[]{
                new Pose2d(26.25, -23.5, Math.toRadians(90)),
                new Pose2d(26.25, 0, Math.toRadians(90)),
                new Pose2d(26.25, 23.5, 0)
        };

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                       new PositionCommand(drivetrain, firstInter, drivetrain.getPoseEstimate()),

                        new ParallelCommandGroup(
                                new WaitCommand(10000)
//                                        .andThen(new AutoDeposit(lift, LiftSubsystem.LiftState.LOW))
                        ),
                        new PositionCommand(drivetrain, firstDeposit, firstInter)
                )
        );


        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            totalLoops++;
            telemetry.clearAll();

            CommandScheduler.getInstance().run();

            lift.loop();
            drivetrain.update();



            double loop = System.nanoTime();

            totalLoopTime += loop;
            avgLoopTime = totalLoopTime/totalLoops;

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("AvgLoopTime", avgLoopTime);
            loopTime = loop;
            telemetry.update();
        }
    }
}
