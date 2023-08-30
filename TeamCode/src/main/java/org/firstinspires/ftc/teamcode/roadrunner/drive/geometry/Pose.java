package org.firstinspires.ftc.teamcode.roadrunner.drive.geometry;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.util.Vector;

public class Pose {

    double x,y,heading;

    public Pose(double x, double y, double rot) {
        this.x = x;
        this.y = y;
        this.heading = rot;
    }

    public Vector2D getVector() {
        return new Vector2D(x,y);
    }

    public double getRot() {
        return heading;
    }
}
