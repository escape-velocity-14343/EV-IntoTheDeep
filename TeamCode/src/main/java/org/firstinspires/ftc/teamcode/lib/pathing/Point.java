package org.firstinspires.ftc.teamcode.lib.pathing;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

public class Point {
    public double x;
    public double y;
    public double heading;


    public Point(double x, double y, double rot) {
        this.x = x;
        this.y = y;
        this.heading = rot;
    }

    public Point setX(double x) {
        this.x = x;
        return this;
    }

    public Point setY(double y) {
        this.y = y;
        return this;
    }

    public Point setHeading(double heading) {
        this.heading = heading;
        return this;
    }

    public Point offset(Point p2) {
        this.x += p2.x;
        this.y += p2.y;
        this.heading += p2.heading;
        return this;
    }

    public Point difference(Point p2) {
        return offset(p2.reverseX().reverseY().reverseHeading());
    }

    public Point offsetX(double x) {
        return this.offset(new Point(x, 0, 0));
    }

    public Point offsetY(double y) {
        return this.offset(new Point(0, y, 0));
    }

    public Point offsetHeading(double rot) {
        return this.offset(new Point(0, 0, rot));
    }

    public Point offset(double x, double y) {
        return this.offset(new Point(x, y, 0));
    }

    public Point offset(double x, double y, double rot) {
        return this.offset(new Point(x, y, rot));
    }


    public Point reverseX() {
        return new Point(-this.x, this.y, this.heading);
    }

    public Point reverseY() {
        return new Point(this.x, -this.y, this.heading);
    }

    public Point reverseHeading() {
        return new Point(this.x, this.y, -this.heading);
    }

    public static Point fromPose2d(Pose2d pose) {
        return new Point(pose.getX(), pose.getY(), pose.getRotation().getRadians());
    }

    public Pose2d toPose2d() {
        return new Pose2d(this.x, this.y, new Rotation2d(this.heading));
    }

    public Vector2d toVector2d() {
        return new Vector2d(this.x, this.y);
    }

    public static double distance(Point p1, Point p2) {
        return Math.sqrt(Math.pow((p1.x - p2.x), 2) + Math.pow((p1.y - p2.y), 2));

    }

}