package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Framework.IPeriodicTask;
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
    private double redSpeakerY = 5547.87;

    public AutoShot() {
        launcherLinearHandle = Subsystems.launcher.linearVelocity.getHandle("autoShot");
        launcherCurveHandle = Subsystems.launcher.spinVelocity.getHandle("autoShot");
        launcherTiltHandle = Subsystems.launcher.tilt.getHandle("autoShot");

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
        return Math.abs(Subsystems.tracking.shotTargetX - 320) < 120;
    }

    public double distance() {
        return Math.sqrt(Math.pow(Math.abs(redSpeakerX - Subsystems.tracking.robotX), 2)+Math.pow(Math.abs(redSpeakerY - Subsystems.tracking.robotY), 2));
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

    public void fireWhenReady() {
        waitingToLaunch = true;
    }

    public void fullAutoLaunch() {
        setupLauncher(distance());
        fireWhenReady();
    }

    public void onStart(RunContext ctx) {
        waitingToLaunch = false;
    }

    public boolean finished() {
        return !waitingToLaunch;
    }

    public void onLoop(RunContext ctx) {
        if(waitingToLaunch) {
            //setupLauncher(distance());
            if(aligned() && Subsystems.launcher.tiltFinished()) {
                Subsystems.launcher.launchNote();
            } 
            if(Subsystems.launcher.finished()) {
                Subsystems.launcher.stopLauncher();
                waitingToLaunch = false;
            }
        }
    }

    public void publishTelemetry() {
        Subsystems.telemetry.pushBoolean("autoShot.aligned", aligned());
        Subsystems.telemetry.pushDouble("autoShot.distance", distance());
        Subsystems.telemetry.pushDouble("autoShot_tiltVal", linearInterpolate(Constants.AutoShot.tiltMap, distance()));
    }

    public void onStop() {

    }
}
