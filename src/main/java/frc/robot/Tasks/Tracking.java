package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.PoseStreamerClient;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Subsystems;

public class Tracking implements IPeriodicTask {
    
    //TODO: multi-gyro input and fusing with vision
    Rotation2d getFieldRelativeRotation() {
        return Hardware.navX.getRotation2d();
    }

    double getGyroRate() {
        return Math.toRadians(Hardware.navX.getRawGyroZ());
    }

    // --Odometry system(wheel encoder and gyroscope tracking)--
    SwerveDriveOdometry odometry;

    StatusSignal<Double> leftPosition;
    StatusSignal<Double> rightPosition;
    double previousLeftPosition;
    double previousRightPosition;
    double previousHeading; //non-continous

    StatusSignal<Double> leftVelocity;
    StatusSignal<Double> rightVelocity;

    PoseStreamerClient client;

    //public double shotTargetX;
    //public double shotTargetY;

    public double noteTargetX;
    public double noteTargetY;

    public double robotX;
    public double robotY;

    Pose2d robotPose;

    Pose2d cameraPose;
    StructPublisher<Pose2d> fusedPosePublisher;
    StructPublisher<Pose2d> cameraPosePublisher;

    public long lastCameraCorrection;

    /**
     * OdometryDelta
     */
    public class OdometrySnapshot {
        public SwerveModulePosition[] modulePositions;
        public Rotation2d gyroRotation;
        public long timestamp;

        public OdometrySnapshot(SwerveModulePosition[] modulePositions, Rotation2d rotation) {
            this.timestamp = System.nanoTime();

            this.modulePositions = modulePositions.clone();
            this.gyroRotation = rotation;
        }
    }

    LinkedList<OdometrySnapshot> odometrySnapshots;

    void initOdometry() {
        leftPosition = new StatusSignal<>(Double.class, null);
        rightPosition = new StatusSignal<>(Double.class, null);

        //change the update frequency depending on the loop time
        leftPosition.setUpdateFrequency(200);
        rightPosition.setUpdateFrequency(200);

        leftVelocity = new StatusSignal<>(Double.class, null);
        rightVelocity = new StatusSignal<>(Double.class, null);
        //FIXME: convert tracking odometry to swerve kinematics

        leftVelocity.setUpdateFrequency(200);
        rightVelocity.setUpdateFrequency(200);

        odometry = new SwerveDriveOdometry(
            Subsystems.drive.kinematics,
            getFieldRelativeRotation(),
            Subsystems.drive.getModulePositions()
        );
        
        odometrySnapshots = new LinkedList<OdometrySnapshot>();
    }

    void updateOdometry() {
        odometry.update(
            getFieldRelativeRotation(),
            Subsystems.drive.getModulePositions()
        );
        odometrySnapshots.add(new OdometrySnapshot(
            Subsystems.drive.getModulePositions(),
            getFieldRelativeRotation()
        ));
        if(odometrySnapshots.size() > 100) {
            odometrySnapshots.pop();
        }
    }

    public void fuseCameraPose(Pose2d cameraPose, long frame_timestamp) {
        LinkedList<OdometrySnapshot> tempSnapshots = (LinkedList<OdometrySnapshot>)odometrySnapshots.clone();
        if(tempSnapshots.size() < 1) {
            odometry.resetPosition(
                Hardware.navX.getRotation2d(), 
                Subsystems.drive.getModulePositions(),
                cameraPose);
            lastCameraCorrection = System.nanoTime();
            return;
        }
        OdometrySnapshot closestSnapshot = tempSnapshots.pop();
        if(closestSnapshot.timestamp > frame_timestamp) {
            //dont fuse the pose
            //return;
        }
        while (frame_timestamp > closestSnapshot.timestamp) {
            closestSnapshot = tempSnapshots.pop();
        }
        //no sensor snapshots after the camera pose, set the pose now
        if(closestSnapshot.timestamp < frame_timestamp) {
            odometry.resetPosition(
                Hardware.navX.getRotation2d(), 
                Subsystems.drive.getModulePositions(),
                cameraPose);
            lastCameraCorrection = System.nanoTime();
            Subsystems.telemetry.pushBoolean("tracking_resetFusion", true);
            return;
        }

        odometry.resetPosition(
                closestSnapshot.gyroRotation, 
                closestSnapshot.modulePositions,
                cameraPose);
        int numFusedSnapshots = 0;
        while(tempSnapshots.size() > 0) {
            OdometrySnapshot snapshot = tempSnapshots.pop();
            odometry.update(
                snapshot.gyroRotation,
                snapshot.modulePositions
            );
            numFusedSnapshots++;
        }
        Subsystems.telemetry.pushDouble("tracking.numFusedSnapshots", numFusedSnapshots);
        Subsystems.telemetry.pushBoolean("tracking_resetFusion", false);
        lastCameraCorrection = System.nanoTime();
    }

    public boolean poseGood() {
        return (System.nanoTime() - lastCameraCorrection) / 1000000 < 400;
    }

    public void setOdometryPose(Pose2d pose) {
        odometry.resetPosition(getFieldRelativeRotation(), 
            Subsystems.drive.getModulePositions(),
            pose
        );
    }

    public ChassisSpeeds getChassisSpeeds() {
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
            leftVelocity.getValue()  * Constants.Drive.rotorToMeters,
            rightVelocity.getValue() * Constants.Drive.rotorToMeters
        );

        return Subsystems.drive.kinematics.toChassisSpeeds();
    }

    //general getters

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.disabled);
            add(RunContext.teleoperated);
            add(RunContext.autonomous);
        }};
    }

    public void onStart(RunContext ctx) {
        fusedPosePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getStructTopic("tracking_FusedPose", Pose2d.struct).publish();

        cameraPosePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getStructTopic("tracking_CameraPose", Pose2d.struct).publish();

        initOdometry();
        client = new PoseStreamerClient("vision.local", 8833);
        client.start();
        client.requestPose(1, 1, (frame) -> {
            for(int i=0; i<frame.values.size(); i++) {
                Subsystems.telemetry.pushDouble("tracking.cameraPose" + i, frame.values.get(i));
            }
            Subsystems.telemetry.pushDouble("tracking.cameraPoseDeltaT", (System.nanoTime() - frame.timestamp)/1000000.0);
            robotX = frame.values.get(0);
            robotY = frame.values.get(1);
            cameraPose = new Pose2d(robotX/1000.0, robotY/1000.0, new Rotation2d(frame.values.get(3) + Math.PI));
            cameraPosePublisher.set(cameraPose);

            double minCornerDist = (frame.values.get(5) > 1)? Constants.Tracking.minCornerDistMulti : Constants.Tracking.minCornerDistSingle;

            if(frame.values.get(5) > 0 && frame.values.get(4) < 1.25 && (frame.values.get(2) < Constants.Tracking.zClamp) && frame.values.get(7) > minCornerDist) { //require at least 2 tags
                synchronized(odometry) {
                    fuseCameraPose(cameraPose, frame.timestamp /*- 300000000*/);
                }
            }
        });

        /*client.requestPose(2, 0, (frame) -> {
            for(int i=0; i<frame.values.size(); i++) {
                Subsystems.telemetry.pushDouble("tracking.tag" + frame.id + "Pose" + i, frame.values.get(i));
            }
            Subsystems.telemetry.pushDouble("tracking.tagPoseDeltaT", (System.nanoTime() - frame.timestamp)/1000000.0);
            shotTargetX = frame.values.get(0);
            shotTargetY = frame.values.get(1);
        });*/

        client.requestPose(3, 1, (frame) -> {
            Subsystems.telemetry.pushDouble("tracking.notePoseX", frame.values.get(0) -320);
            Subsystems.telemetry.pushDouble("tracking.notePoseY", frame.values.get(1) -240);
            noteTargetX = frame.values.get(0) - 320;
            noteTargetY = frame.values.get(1) - 240;
        });

        /*client.requestPose(4, 0, (frame) -> {
            
        });*/

        client.awaiting_clock_request = true;
    }

    public void publishTelemetry() {
        Subsystems.telemetry.pushDouble("tracking_odometryX", odometry.getPoseMeters().getX());
        Subsystems.telemetry.pushDouble("tracking_odometryY", odometry.getPoseMeters().getY());
        Subsystems.telemetry.pushDouble("tracking_odometryRadians", odometry.getPoseMeters().getRotation().getRadians());
        Subsystems.telemetry.pushDouble("tracking_leftPosition", leftPosition.getValue());
        Subsystems.telemetry.pushDouble("tracking_rightPosition", rightPosition.getValue());
        Subsystems.telemetry.pushDouble("tracking_leftVelocity", leftVelocity.getValue());
        Subsystems.telemetry.pushDouble("tracking_rightVelocity", rightVelocity.getValue());
        Subsystems.telemetry.pushDouble("tracking_theta", getFieldRelativeRotation().getDegrees());
        Subsystems.telemetry.pushDouble("tracking_omegaRadiansPerSecondGyro", Math.toRadians(Hardware.navX.getRawGyroZ()));
        Subsystems.telemetry.pushBoolean("tracking_poseGood", poseGood());
    }

    public void onLoop(RunContext ctx) {
        BaseStatusSignal.refreshAll(leftPosition, rightPosition, rightVelocity, leftVelocity);

        synchronized(odometry) {
            updateOdometry();
        }
        fusedPosePublisher.set(odometry.getPoseMeters());

        //lights to tell technicians whether the robot can see the tags
        if(DriverStation.isFMSAttached() && DriverStation.isDisabled() && DriverStation.getMatchTime() > 10) {
            if((System.nanoTime() - lastCameraCorrection) < 150000000) {
                Subsystems.illumination.setStatic((byte)0, 0, 90, 0);
                Subsystems.illumination.setStatic((byte)1, 0, 90, 0);
            } else {
                Subsystems.illumination.setStatic((byte)0, 90, 00, 0);
                Subsystems.illumination.setStatic((byte)1, 90, 00, 0);
            }
        }
    }

    public void onStop() {}
}
