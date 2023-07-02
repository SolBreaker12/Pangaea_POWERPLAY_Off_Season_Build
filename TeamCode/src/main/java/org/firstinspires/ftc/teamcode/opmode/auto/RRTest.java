package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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

    private ElapsedTime timer;
    private double loopTime;
    private double endtime;

    @Override
    public void runOpMode() throws InterruptedException {

//      Reset instance of the CommandScheduler
        CommandScheduler.getInstance().reset();

        robot.initRobot(hardwareMap);
        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        rrDrive = new SampleMecanumDrive(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(rrDrive);

//      Set the robots starting Pose to 0, 0, 0
        mecanumSubsystem.drive.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

//      Trajectory spline to move to Pose 30, 30, and rotate 180 degrees
        Trajectory testSpline1 = mecanumSubsystem.drive.trajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(30,30), 180)
                .build();

//      Schedule commands
        CommandScheduler.getInstance().schedule(
//          Execute in order
            new SequentialCommandGroup(
//                  Run created spline Trajectory
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
        }
    }
}
