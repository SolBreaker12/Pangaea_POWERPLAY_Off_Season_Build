package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.function.DoubleSupplier;

public class MecanumSubsystem extends SubsystemBase  {
    public final SampleMecanumDrive drive;

    public MecanumSubsystem(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    public Command followTrajectory(Trajectory trajectory) {
        return new FunctionalCommand(
                () -> drive.followTrajectory(trajectory),
                drive::update, e -> {},
                () -> !drive.isBusy(),
                this
        );
    }

    public Command followTrajectorySequence(TrajectorySequence trajectory) {
        return new FunctionalCommand(
                () -> drive.followTrajectorySequence(trajectory),
                drive::update, e -> {},
                () -> !drive.isBusy(),
                this
        );
    }

    public Command driveRobotCentric(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        return new RunCommand(
                () -> drive.setWeightedDrivePower(
                        new Pose2d(
                                leftX.getAsDouble(),
                                leftY.getAsDouble(),
                                -rightX.getAsDouble()
                        )
                ), this
        );
    }

    public Command driveFieldCentric(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        Vector2d rotated = new Vector2d(-leftY.getAsDouble(), -leftX.getAsDouble()).rotated(
                -drive.getPoseEstimate().getHeading()
        );

        return new RunCommand(
                () -> drive.setWeightedDrivePower(
                        new Pose2d(
                                rotated.getX(),
                                rotated.getY(),
                                -rightX.getAsDouble()
                        )
                ), this
        );
    }

    public Command driveFieldCentric(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier gyroAngle) {
        Vector2d rotated = new Vector2d(-leftY.getAsDouble(), -leftX.getAsDouble()).rotated(
                -gyroAngle.getAsDouble()
        );

        return new RunCommand(
                () -> drive.setWeightedDrivePower(
                        new Pose2d(
                                rotated.getX(),
                                rotated.getY(),
                                -rightX.getAsDouble()
                        )
                ), this
        );
    }
}
