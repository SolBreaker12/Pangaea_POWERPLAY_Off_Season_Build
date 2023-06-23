package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.MecanumSubsystem;

@Autonomous
public class RRTest extends LinearOpMode {

//  Instantiate robot with Instance of Singleton
    private final static Robot robot = Robot.getInstance();

    private static MecanumSubsystem mecanumSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {

//      Reset instance of the CommandScheduler
        CommandScheduler.getInstance().reset();

        robot.InitRobot(hardwareMap);
        robot.enabled = true;

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
                    mecanumSubsystem.followTrajectory(testSpline1)
            )
        );

        while(opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}
