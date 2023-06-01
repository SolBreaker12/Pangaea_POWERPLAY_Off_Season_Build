package org.firstinspires.ftc.teamcode.common.commandbase.base;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class ClawCommand extends CommandBase {
    public ClawCommand(LiftSubsystem lift, LiftSubsystem.ClawState state) {
        lift.update(state);
    }

}
