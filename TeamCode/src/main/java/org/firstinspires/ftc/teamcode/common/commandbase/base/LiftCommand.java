package org.firstinspires.ftc.teamcode.common.commandbase.base;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class LiftCommand extends CommandBase {
    public LiftCommand(LiftSubsystem lift, LiftSubsystem.LiftState state) {
        lift.update(state);
    }

}