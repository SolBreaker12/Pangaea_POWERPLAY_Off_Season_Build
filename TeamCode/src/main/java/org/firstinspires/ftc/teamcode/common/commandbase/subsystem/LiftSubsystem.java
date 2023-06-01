package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import static org.firstinspires.ftc.teamcode.globals.Globals.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;


public class LiftSubsystem extends SubsystemBase {

    private final RobotHardware robot;
    private LiftState liftState = LiftState.RETRACTED;
    private ClawState clawState = ClawState.OPEN;

    private final int allowedError = 20;

    private int targetPosition;
    private int currentPosition;
    private boolean withinTolerance = false;

    static double power;

    private final PIDController liftPID;
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;

    public boolean hasCone = false;

    public enum LiftState {
        GROUND,
        LOW,
        MEDIUM,
        HIGH,
        RETRACTED
    }

    public enum ClawState {
        CLOSED,
        OPEN
    }

    public LiftSubsystem(RobotHardware robot) {
        this.robot = robot;
        kP = 0;
        kI = 0;
        kD = 0;
        kF = 0;

        liftPID = new PIDController(kP, kI, kD);

        hasCone = false;
    }

    /*public void update(LiftState state) {
        liftState = state;
        switch (liftState) {
            case RETRACTED:
                targetPosition = LIFT_RETRACTED_POS;
            case GROUND:
                targetPosition = LIFT_GROUND_POS;
            case LOW:
                targetPosition = LIFT_LOW_POS;
            case MEDIUM:
                targetPosition = LIFT_MED_POS;
            case HIGH:
                targetPosition = LIFT_HIGH_POS;
        }
        setTargetPosition(targetPosition);
    }*/

    public void update(ClawState state) {
        clawState = state;
        switch (state) {
            case OPEN:
                robot.servo1.setPosition(ClawState.OPEN.ordinal());
                robot.servo2.setPosition(ClawState.OPEN.ordinal());
            case CLOSED:
                robot.servo1.setPosition(ClawState.CLOSED.ordinal());
                robot.servo2.setPosition(ClawState.CLOSED.ordinal());
        }
    }

    public void update(LiftState state) {
        liftState = state;
        switch (liftState) {
            case RETRACTED:
                targetPosition = LIFT_RETRACTED_POS;
            case GROUND:
                targetPosition = LIFT_GROUND_POS;
            case LOW:
                targetPosition = LIFT_LOW_POS;
            case MEDIUM:
                targetPosition = LIFT_MED_POS;
            case HIGH:
                targetPosition = LIFT_HIGH_POS;
        }
    }
    public void loop() {
        withinTolerance = Math.abs(getCurrentPosition() - targetPosition) < allowedError;

        liftPID.setPID(kP, kI, kD);

        double PowerPID = liftPID.calculate(currentPosition, targetPosition);
        double PowerFF = kF * Math.signum(targetPosition - currentPosition);

        power = Range.clip(PowerPID + PowerFF, -1, 1);
    }

    public void read() {
        setCurrentPosition();
    }

    public void write() {
        robot.liftMotor.set(power);
    }


    public int getCurrentPosition() {
        return robot.liftMotor.getCurrentPosition();
    }

    public void setCurrentPosition() { currentPosition = getCurrentPosition(); }

    public boolean hasCone() { return hasCone; }

    public boolean atTargetPosition() {
        return withinTolerance;
    }
}
