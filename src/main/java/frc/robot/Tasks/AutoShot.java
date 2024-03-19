package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.Map.Entry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    //ParameterHandle<ChassisSpeeds> driveHandle;

    private boolean waitingToLaunch;
    private boolean waitingForAmpWhisker;
    private double redSpeakerX = 16579.34 / 1000.0;
    private double blueSpeakerX = 0.0;

    private double speakerX = redSpeakerX;
    private double redSpeakerY = 5547.87 / 1000.0;

    Pose2d ampPose;

    private double targetX;
    private double targetY;

    private double angleOffset;

    private boolean redAlliance;

    PIDController alignmentController;

    public AutoShot() {
        launcherLinearHandle = Subsystems.launcher.linearVelocity.getHandle("autoShot");
        launcherCurveHandle = Subsystems.launcher.spinVelocity.getHandle("autoShot");
        launcherTiltHandle = Subsystems.launcher.tilt.getHandle("autoShot");

        //driveHandle = Subsystems.differentialDrive.wantedChassisSpeeds.getHandle("autoShot");

        alignmentController = new PIDController(8.0, 0.0, 0.0);

        SmartDashboard.putNumber("autoShot_simDistance", 0);

        targetX = redSpeakerX;
        targetY = redSpeakerY;

        redAlliance = true;

        SmartDashboard.putNumber("autoShot_angleOffset", 0.0);
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.disabled);
            add(RunContext.teleoperated);
            add(RunContext.autonomous);
        }};
    }

    public boolean aligned() {
        return Math.abs(Subsystems.tracking.odometry.getPoseMeters().getRotation().getRadians() - angleToTarget()) < 0.1;
    }

    public double distance(double x, double y) {
        return Math.sqrt(Math.pow(Math.abs(x - (Subsystems.tracking.odometry.getPoseMeters().getX())), 2)+Math.pow(Math.abs(y - (Subsystems.tracking.odometry.getPoseMeters().getY())), 2));
    }

    public double angleToTarget() {
        double dX = targetX - (Subsystems.tracking.odometry.getPoseMeters().getX());
        double dY = targetY - (Subsystems.tracking.odometry.getPoseMeters().getY());

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

    private void setupShot(double tilt, double linearVelocity) {
        launcherLinearHandle.takeControl(false);
        launcherCurveHandle.takeControl(false);
        launcherTiltHandle.takeControl(false);

        launcherLinearHandle.set(linearVelocity);
        launcherCurveHandle.set(0.0);
        launcherTiltHandle.set(tilt);
        Subsystems.telemetry.pushDouble("autoShot_calculated_velocity", linearVelocity);
        Subsystems.telemetry.pushDouble("autoShot_calculated_tilt", tilt);
    }

    public void setupSpeakerShot(double distance) {
        setupShot(linearInterpolate(Constants.AutoShot.tiltMap, distance*1000) + angleOffset, linearInterpolate(Constants.AutoShot.velocityMap, distance*1000));
    }

    private Pose2d calculateWhisker(Pose2d target, double whiskerLength) {
        double whiskerX, whiskerY;

        //offset the pose by the whisker length in the direction that the trap faces
        whiskerX = whiskerLength * Math.cos(target.getRotation().getRadians());
        whiskerY = whiskerLength * Math.sin(target.getRotation().getRadians());

        return new Pose2d(target.getX() + whiskerX, target.getY() + whiskerY,target.getRotation());
    }

    public void setupAmpShot() {
        setupShot(Constants.Launcher.ampTiltPosition, Constants.Launcher.ampVelocity);

        if(redAlliance) {
            ampPose = Constants.AutoShot.redAmpPose;
        } else {
            ampPose = Constants.AutoShot.blueAmpPose;
        }

        Pose2d whiskerPose = calculateWhisker(ampPose, Constants.AutoShot.ampSecondWhiskerLength);

        Subsystems.autoPilot.setTarget(whiskerPose, true);
        Subsystems.autoPilot.driveToPoint();
        waitingForAmpWhisker = true;
    }

    private Pose2d closestTrap() {
        double shortestDistance = -1;
        int closestIdx = -1;
        Pose2d closestPose = new Pose2d();

        Set<Entry<Integer, Pose2d>> pointSet;

        if(redAlliance) {
            pointSet = Constants.AutoShot.redTrapFiringPoints.entrySet();
        } else {
            pointSet = Constants.AutoShot.blueTrapFiringPoints.entrySet();
        }

        for (Entry<Integer, Pose2d> trapPose : pointSet) {
            double trapDistance = distance(trapPose.getValue().getX(), trapPose.getValue().getY());
            if(trapDistance < shortestDistance ||
                shortestDistance < 0) {
                shortestDistance = trapDistance;
                closestIdx = trapPose.getKey();
                closestPose = trapPose.getValue();
            }

        }

        Subsystems.telemetry.pushDouble("autoShot_trap_closestIdx", closestIdx);
        Subsystems.telemetry.pushDouble("autoShot_trap_distance", shortestDistance);

        return closestPose;
    }

    public void setupTrapShot() {
        setupShot(Constants.Launcher.trapTiltPosition, Constants.Launcher.trapVelocity);

        Pose2d trapPose = closestTrap();

        double whiskerX, whiskerY;

        //offset the pose by the whisker length in the direction that the trap faces
        whiskerX = Constants.AutoShot.trapWhiskerLength * Math.cos(trapPose.getRotation().getRadians());
        whiskerY = Constants.AutoShot.trapWhiskerLength * Math.sin(trapPose.getRotation().getRadians());

        Pose2d whiskerPose = new Pose2d(trapPose.getX() + whiskerX, trapPose.getY() + whiskerY,closestTrap().getRotation());

        Subsystems.autoPilot.setTarget(whiskerPose, true);
        Subsystems.autoPilot.driveToPoint();
    }

    public void fireWhenReady() {
        waitingToLaunch = true;
    }

    public void fullAutoLaunch() {
        targetX = speakerX;
        targetY = redSpeakerY;
        setupSpeakerShot(distance(targetX, targetY));
        Subsystems.autoPilot.setTarget(new Pose2d(speakerX, redSpeakerY, new Rotation2d()), true);
        Subsystems.autoPilot.facePoint();
        fireWhenReady();
    }

    public void cancelAutoLaunch() {
        waitingToLaunch = false;
        Subsystems.launcher.stopLauncher();
        Subsystems.autoPilot.disable();
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
                redAlliance = true;
            } else {
                speakerX = blueSpeakerX;
                redAlliance = false;
            }
        }

        if(waitingToLaunch) {
            setupSpeakerShot((distance(targetX, targetY)));
            if(aligned() && Subsystems.launcher.tiltFinished()) {
                Subsystems.launcher.launchNote();
            } 
            if(Subsystems.launcher.finished()) {
                cancelAutoLaunch();
                Subsystems.illumination.setStatic((byte)0, 0, 130, 0);
                if(Constants.AutoShot.dropTiltAfterSpeakerShot) {
                    launcherTiltHandle.set(Constants.Launcher.tiltDefaultPosition);
                }
            }
        }

        if(waitingForAmpWhisker) {
            if(Subsystems.autoPilot.finished()) {
                Pose2d secondStagePose = calculateWhisker(ampPose, Constants.AutoShot.ampWhiskerLength);
                Subsystems.autoPilot.disable();
                Subsystems.autoPilot.setTarget(secondStagePose, true);
                Subsystems.autoPilot.driveToPoint();
                waitingForAmpWhisker = false;
            }
        }

        angleOffset = SmartDashboard.getNumber("autoShot_angleOffset", angleOffset);
    }

    public void publishTelemetry() {
        Subsystems.telemetry.pushBoolean("autoShot_aligned", aligned());
        Subsystems.telemetry.pushDouble("autoShot_distance", distance(targetX, targetY));
        //Subsystems.telemetry.pushDouble("autoShot_tiltVal", linearInterpolate(Constants.AutoShot.tiltMap, distance()));
        Subsystems.telemetry.pushDouble("autoShot_angleToTarget", angleToTarget());
    }

    public void onStop() {

    }
}
