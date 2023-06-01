package org.firstinspires.ftc.teamcode.common.commandbase.base;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DrivetrainSubsystem;

public class TrajectoryFollowerCommand extends CommandBase {

    DrivetrainSubsystem drive;
    Trajectory trajectory;

    public TrajectoryFollowerCommand(DrivetrainSubsystem drive, Trajectory trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        drive.followTrajectoryAsync(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }


}
