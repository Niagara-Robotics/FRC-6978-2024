package frc.robot.Framework;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RamseteController {
    Vector2D currentPose;
    Vector2D target;

    double beta;
    double zeta;

    void setActual(Pose2d pose) {
        currentPose.x = pose.getX();
        currentPose.y = pose.getY();
        currentPose.theta = pose.getRotation().getRadians();
    }

    void setTarget(Vector2D target) {
        this.target = target;
    }

    ChassisSpeeds evaluate() {
        double dx = target.x - currentPose.x;
        double dy = target.y - currentPose.y;

        // calculate ex, ey
        double ex = dx*Math.cos(currentPose.theta) + dy*Math.sin(currentPose.theta);
        double ey = dx*-Math.sin(currentPose.theta) + dy*Math.cos(currentPose.theta);
        double etheta = target.theta - currentPose.theta;

        double k = 2 * zeta * Math.sqrt(Math.pow(target.omega,2) + beta * Math.pow(target.v, 2));

        double v = target.v * Math.cos(etheta) + k * ex;
        double omega = target.omega + k * etheta + ((beta*target.v*Math.sin(etheta)*ey)/etheta);

        return new ChassisSpeeds(v, 0, omega);
    }
}
