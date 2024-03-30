package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.PIDController;
import frc.robot.Framework.ParameterHandle;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Subsystems;

public class AutoPilot implements IPeriodicTask {
    public enum AutoPilotMode {
        facePoint,
        driveToPoint,
        driveDistance,
        disabled,
    }

    private AutoPilotMode currentMode;

    private ParameterHandle<ChassisSpeeds> driveHandle;

    //private double targetX;
    //private double targetY;

    private Pose2d targetPose;
    private StructPublisher<Pose2d> targetPosePublisher;

    private boolean reverse;
    private boolean hitPoint;

    PIDController alignmentController;
    PIDController distanceController;

    public AutoPilot() {
        driveHandle = Subsystems.differentialDrive.wantedChassisSpeeds.getHandle("AutoPilot");

        alignmentController = new PIDController(Constants.AutoPilot.anglekP, 0.0, 0.0);
        alignmentController.setLimit(Constants.AutoPilot.angularVelocityLimit);

        distanceController = new PIDController(Constants.AutoPilot.distancekP, 0, 0);
        distanceController.setLimit(Constants.AutoPilot.linearVelocityLimit);

        targetPosePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getStructTopic("autoPilot_TargetPose", Pose2d.struct).publish();

        targetPose = new Pose2d();
    }

    public double angleToTarget() {
        double dX = targetPose.getX() - (Subsystems.tracking.odometry.getPoseMeters().getX());
        double dY = targetPose.getY() - (Subsystems.tracking.odometry.getPoseMeters().getY());

        double angle = Math.atan2(dY, dX);
        if(reverse) {
            if(angle < 0) angle = angle + Math.PI;
            else if(angle > 0) angle = -(Math.PI - angle);
        }
        return angle;
    }

    public double distance() {
        return Math.sqrt(Math.pow(Math.abs(targetPose.getX() - (Subsystems.tracking.odometry.getPoseMeters().getX())), 2)+Math.pow(Math.abs(targetPose.getY() - (Subsystems.tracking.odometry.getPoseMeters().getY())), 2)) *1.0;
    }

    double closestAngleDelta(double targetRadians, double actualRadians) {
        double delta = targetRadians - actualRadians;
        //Subsystems.telemetry.pushDouble("rawDelta", delta);
        if(delta > Math.PI) delta = -(delta - Math.PI);
        if(delta < -Math.PI) delta = (delta + Math.PI*2);
        
        return delta;
    }

    //MUST be disabled to change target coordinates
    public void setTarget(Pose2d pose, boolean reverse) {
        if(currentMode != AutoPilotMode.disabled) return;
        targetPose = pose;
        this.reverse = reverse;
    }

    public void facePoint() {
        currentMode = AutoPilotMode.facePoint;
        alignmentController.init();
        driveHandle.takeControl(false);
        hitPoint = false;
    }

    public void driveToPoint() {
        currentMode = AutoPilotMode.driveToPoint;
        alignmentController.init();
        distanceController.init();
        driveHandle.takeControl(false);
        hitPoint = false;
    }

    public void releaseControl() {
        driveHandle.release();
    }

    public void disable() {
        currentMode = AutoPilotMode.disabled;
        releaseControl();
    }

    public void onStart(RunContext ctx) {
        currentMode = AutoPilotMode.disabled;
    }

    public void onLoop(RunContext ctx) {
        double angleDelta;
        if(distance() < Constants.AutoPilot.distanceThreshold && currentMode == AutoPilotMode.driveToPoint || hitPoint) {//face target pose
            angleDelta = -closestAngleDelta(targetPose.getRotation().getRadians(), Subsystems.tracking.getPose().getRotation().getRadians());
            hitPoint= true;
        }
        else
            angleDelta = -closestAngleDelta(angleToTarget(), Subsystems.tracking.getPose().getRotation().getRadians());;

        Subsystems.telemetry.pushDouble("autoPilot_angleDelta", angleDelta);

        double angularVelocity = alignmentController.process(angleDelta);

        if(Math.abs(angularVelocity) < Constants.AutoPilot.angularVelocityThreshold) {
            angularVelocity = 0;
        }

        double linearVelocityCap = 1 - (Math.abs(angleDelta) / Constants.AutoPilot.deltaStopPoint);
        if(linearVelocityCap < 0) linearVelocityCap = 0;
        linearVelocityCap *= Constants.AutoPilot.linearVelocityLimit;

        Subsystems.telemetry.pushDouble("autoPilot_linearVelocityCap", linearVelocityCap);

        distanceController.setLimit(linearVelocityCap);
        double linearVelocity = distanceController.process(distance());
        if(!reverse) linearVelocity = -linearVelocity;

        if(distance() < Constants.AutoPilot.distanceThreshold && currentMode == AutoPilotMode.driveToPoint || hitPoint)
            linearVelocity = 0;

        switch (currentMode) {
            case facePoint:
                driveHandle.set(new ChassisSpeeds(0,0,angularVelocity));
                break;
            case driveToPoint:
                driveHandle.set(new ChassisSpeeds(linearVelocity,0,angularVelocity));
                break;
            default:
                break;
        }
    }

    public boolean finished() {
        return hitPoint;
    }

    public void publishTelemetry() {
        targetPosePublisher.set(targetPose);
        Subsystems.telemetry.pushBoolean("autoPilot_hitPoint", hitPoint);
    }

    public void onStop() {}

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.disabled);
            add(RunContext.teleoperated);
            add(RunContext.autonomous);
        }};
    }
}
