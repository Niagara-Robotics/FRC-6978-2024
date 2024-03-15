package frc.robot.Framework;

import edu.wpi.first.math.geometry.Pose2d;

public class Vector2D {
    double x;
    double y;
    double v;
    double omega;
    double theta;

    public Vector2D(Pose2d pose, double omega, double v) {
        x = pose.getX();
        y = pose.getY();
        this.v = v;
        this.omega = omega;
    }
}
