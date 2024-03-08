package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.PIDController;
import frc.robot.Framework.ParameterHandle;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Subsystems;

public class AutoShot implements IPeriodicTask {
    ParameterHandle<Double> launcherLinearHandle;
    ParameterHandle<Double> launcherCurveHandle;
    ParameterHandle<Double> launcherTiltHandle;

    private boolean waitingToLaunch;
    private double redSpeakerX = 16579.34;
    private double blueSpeakerX = 0.0;

    private double speakerX = redSpeakerX;
    private double redSpeakerY = 5547.87;

    PIDController alignmentController;

    public AutoShot() {
        launcherLinearHandle = Subsystems.launcher.linearVelocity.getHandle("autoShot");
        launcherCurveHandle = Subsystems.launcher.spinVelocity.getHandle("autoShot");
        launcherTiltHandle = Subsystems.launcher.tilt.getHandle("autoShot");

        alignmentController = new PIDController(8.0, 0.0, 0.0);

        SmartDashboard.putNumber("autoShot_simDistance", 0);
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.disabled);
            add(RunContext.teleoperated);
            add(RunContext.autonomous);
        }};
    }

    public boolean aligned() {
        return Math.abs(Subsystems.tracking.odometry.getPoseMeters().getRotation().getRadians() - angleToTarget()) < 0.1 && 
        (System.nanoTime() - Subsystems.tracking.lastCameraCorrection) < 600000000;
    }

    public double distance() {
        return Math.sqrt(Math.pow(Math.abs(speakerX - (Subsystems.tracking.odometry.getPoseMeters().getX()*1000.0)), 2)+Math.pow(Math.abs(redSpeakerY - (Subsystems.tracking.odometry.getPoseMeters().getY()*1000.0)), 2)) *1.0;
    }

    public double angleToTarget() {
        double dX = speakerX - (Subsystems.tracking.odometry.getPoseMeters().getX()*1000.0);
        double dY = redSpeakerY - (Subsystems.tracking.odometry.getPoseMeters().getY()*1000.0);

        double angle = Math.atan2(dY, dX);
        if(angle < 0) angle = angle + Math.PI;
        else if(angle > 0) angle = -(Math.PI - angle);
        return angle;
    }

    public double linearInterpolate(HashMap<Double, Double> map, double target) {
        double closestAbove = target, closestBelow = target;
        for (double key : map.keySet()) {
            if(key == target) {
                return map.get(target);
            }
            if(key < target) {
                closestBelow = ((target-closestBelow)== 0)? key:((target-key < target-closestBelow)? key:closestBelow);
            }
            if(key > target) {
                closestAbove = ((closestAbove-target)== 0)? key:((key-target < closestAbove-target)? key:closestAbove);
            }
        }
        if(closestAbove == target) {
            return map.get(closestBelow);
        } else if (closestBelow == target) {
            return map.get(closestAbove);
        }
        double l = (target-closestBelow)/(closestAbove-closestBelow);
        return l*(map.get(closestAbove) - map.get(closestBelow)) + map.get(closestBelow);
        //return closestAbove;
    }

    public void setupLauncher(double distance) {
        launcherLinearHandle.takeControl(false);
        launcherCurveHandle.takeControl(false);
        launcherTiltHandle.takeControl(false);

        launcherLinearHandle.set(linearInterpolate(Constants.AutoShot.velocityMap, distance));
        launcherCurveHandle.set(0.0);
        launcherTiltHandle.set(linearInterpolate(Constants.AutoShot.tiltMap, distance));
        Subsystems.telemetry.pushDouble("autoShot_calculated_velocity", linearInterpolate(Constants.AutoShot.velocityMap, distance));
        Subsystems.telemetry.pushDouble("autoShot_calculated_tilt", linearInterpolate(Constants.AutoShot.tiltMap, distance));
    }

    public void setupAmpShot() {
        launcherLinearHandle.takeControl(false);
        launcherCurveHandle.takeControl(false);
        launcherTiltHandle.takeControl(false);

        launcherLinearHandle.set(Constants.Launcher.ampVelocity);
        launcherCurveHandle.set(0.0);
        launcherTiltHandle.set(Constants.Launcher.ampTiltPosition);
    }

    public void setupTrapShot() {
        launcherLinearHandle.takeControl(false);
        launcherCurveHandle.takeControl(false);
        launcherTiltHandle.takeControl(false);

        launcherLinearHandle.set(Constants.Launcher.trapVelocity);
        launcherCurveHandle.set(0.0);
        launcherTiltHandle.set(Constants.Launcher.trapTiltPosition);
    }

    double closestAngleDelta(double targetRadians, double actualRadians) {
        double delta = targetRadians - actualRadians;
        Subsystems.telemetry.pushDouble("rawDelta", delta);
        if(delta > Math.PI) delta = -(delta - Math.PI);
        if(delta < -Math.PI) delta = (delta + Math.PI*2);
        
        return delta;
    }

    public void fireWhenReady() {
        waitingToLaunch = true;
    }

    public void fullAutoLaunch() {
        setupLauncher(distance());
        fireWhenReady();
        alignmentController.init();
        alignmentController.set(0.0);
        alignmentController.setLimit(2.8);
    }

    public void cancelAutoLaunch() {
        waitingToLaunch = false;
        Subsystems.launcher.stopLauncher();
        Subsystems.differentialDrive.driveChassisSpeeds(new ChassisSpeeds(0,0, 0));
    }

    public void onStart(RunContext ctx) {
        waitingToLaunch = false;
    }

    public boolean finished() {
        return !waitingToLaunch;
    }

    public void onLoop(RunContext ctx) {

        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
            if(alliance.get() == Alliance.Red) {
                speakerX = redSpeakerX;
            } else {
                speakerX = blueSpeakerX;
            }
        }

        if(waitingToLaunch) {
            setupLauncher(distance());
            double delta = -closestAngleDelta(Subsystems.autoShot.angleToTarget(), Subsystems.tracking.odometry.getPoseMeters().getRotation().getRadians());
            Subsystems.telemetry.pushDouble("autoAlign.delta", delta);
            double output = alignmentController.process(delta);
            Subsystems.telemetry.pushDouble("autoAlign.output", output);
            Subsystems.differentialDrive.driveChassisSpeeds(new ChassisSpeeds(0,0, output));
            if(aligned() && Subsystems.launcher.tiltFinished() && Math.abs(output) < 0.2) {
                Subsystems.launcher.launchNote();
            } 
            if(Subsystems.launcher.finished()) {
                cancelAutoLaunch();
                Subsystems.illumination.setStatic((byte)0, 0, 130, 0);
            }
        }
    }

    public void publishTelemetry() {
        Subsystems.telemetry.pushBoolean("autoShot.aligned", aligned());
        Subsystems.telemetry.pushDouble("autoShot.distance", distance());
        Subsystems.telemetry.pushDouble("autoShot_tiltVal", linearInterpolate(Constants.AutoShot.tiltMap, distance()));
        Subsystems.telemetry.pushDouble("autoShot.angleToTarget", angleToTarget());
    }

    public void onStop() {

    }
}
