package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous
public class RRTest extends LinearOpMode {

//  Instantiate robot with Instance of Singleton
    private final static Robot robot = Robot.getInstance();

    private static SampleMecanumDrive rrDrive;
    private static MecanumSubsystem mecanumSubsystem;

    private double loopTime;

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().reset();

        robot.initRobot(hardwareMap);
        robot.enabled = true;

//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(8);
//        PhotonCore.enable();

        rrDrive = new SampleMecanumDrive(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(rrDrive);


        mecanumSubsystem.drive.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();



        Trajectory testSpline1 = mecanumSubsystem.drive.trajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(30,30), 180)
                .build();

        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                    new WaitCommand(1000),
                    mecanumSubsystem.followTrajectory(testSpline1)
            )
        );

        while(opModeIsActive()) {
            CommandScheduler.getInstance().run();

            mecanumSubsystem.drive.update();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;

            robot.clearBulkCache();
            telemetry.update();
        }
    }
}
