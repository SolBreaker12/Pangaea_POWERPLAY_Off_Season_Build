package org.firstinspires.ftc.teamcode.globals;

public class Globals {

    public enum Side {
        LEFT,
        RIGHT
    }

    public static final int LIFT_HIGH_POS = (1076 * 3) + 100;
    public static final int LIFT_MED_POS = (1076 * 2) + 100;
    public static final int LIFT_LOW_POS = 1076 + 100;
    public static final int LIFT_GROUND_POS = 10;
    public static final int LIFT_RETRACTED_POS = 0;

    public static final double LIFT_TICKS_PER_INCH = 89.6;

    public static final double CLAW_MAX = (2 * Math.PI) / 3;
    public static final double CLAW_MIN = 0;

    public static Side SIDE = Side.LEFT;
    public static boolean AUTO = false;
    public static boolean USING_IMU = true;
}
