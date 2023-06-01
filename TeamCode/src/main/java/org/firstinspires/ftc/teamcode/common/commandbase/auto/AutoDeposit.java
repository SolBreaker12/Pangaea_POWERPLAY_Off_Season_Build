package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.base.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.base.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class AutoDeposit extends SequentialCommandGroup {

    public AutoDeposit(LiftSubsystem lift, LiftSubsystem.LiftState state) {
        super(
                new LiftCommand(lift, state),
                new WaitUntilCommand(lift::atTargetPosition),
                new ClawCommand(lift, LiftSubsystem.ClawState.OPEN)
        );

    }
}
