package org.firstinspires.ftc.teamcode.common.commandbase.base;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DrivetrainSubsystem;

public class PositionCommand extends CommandBase {

    private final DrivetrainSubsystem drive;
    private static Trajectory trajectory;

    private static Pose2d endPose;
    private static Pose2d startPose;

    public PositionCommand(DrivetrainSubsystem drive, Pose2d endPose, Pose2d startPose) {
        this.drive = drive;
        PositionCommand.endPose = endPose;
        PositionCommand.startPose = startPose;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        trajectory = drive.drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(endPose)
                .build();
        drive.followTrajectoryAsync(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }
}