package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Stack;
import java.util.Vector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // --Odometry system(wheel encoder and gyroscope tracking)--
    DifferentialDriveOdometry odometry;

    StatusSignal<Double> leftPosition;
    StatusSignal<Double> rightPosition;
    double previousLeftPosition;
    double previousRightPosition;
    double previousHeading; //non-continous

    StatusSignal<Double> leftVelocity;
    StatusSignal<Double> rightVelocity;

    PoseStreamerClient client;

    public double shotTargetX;
    public double shotTargetY;

    public double noteTargetX;
    public double noteTargetY;

    public double robotX;
    public double robotY;

    Pose2d robotPose;

    Pose2d cameraPose;
    StructPublisher<Pose2d> posePublisher;

    public long lastCameraCorrection;

    /**
     * OdometryDelta
     */
    public class OdometrySnapshot {
        public double leftDistance;
        public double rightDistance;
        public Rotation2d gyroRotation;
        public long timestamp;

        public OdometrySnapshot(double leftDistance, double rightDistance, Rotation2d rotation) {
            this.timestamp = System.nanoTime();

            this.leftDistance = leftDistance;
            this.rightDistance = rightDistance;
            this.gyroRotation = rotation;
        }
    }

    LinkedList<OdometrySnapshot> odometrySnapshots;

    void initOdometry() {
        leftPosition = Hardware.leftDriveLeader.getPosition();
        rightPosition = Hardware.rightDriveLeader.getPosition();

        //change the update frequency depending on the loop time
        leftPosition.setUpdateFrequency(200);
        rightPosition.setUpdateFrequency(200);

        leftVelocity = Hardware.leftDriveLeader.getVelocity();
        rightVelocity = Hardware.rightDriveLeader.getVelocity();

        leftVelocity.setUpdateFrequency(200);
        rightVelocity.setUpdateFrequency(200);

        odometry = new DifferentialDriveOdometry(
            Hardware.navX.getRotation2d(),
            leftPosition.getValue() * Constants.Drive.rotorToMeters,
            rightPosition.getValue() * Constants.Drive.rotorToMeters,
            new Pose2d(0, 0, new Rotation2d())
        );
        
        odometrySnapshots = new LinkedList<OdometrySnapshot>();
    }

    void updateOdometry() {
        odometry.update(
            getFieldRelativeRotation(),
            leftPosition.getValue() * Constants.Drive.rotorToMeters, 
            rightPosition.getValue() * Constants.Drive.rotorToMeters
        );
        odometrySnapshots.add(new OdometrySnapshot(
            leftPosition.getValue() * Constants.Drive.rotorToMeters, 
            rightPosition.getValue() * Constants.Drive.rotorToMeters, 
            getFieldRelativeRotation()
        ));
        if(odometrySnapshots.size() > 100) {
            odometrySnapshots.pop();
        }
    }

    public void fuseCameraPose(Pose2d cameraPose, long frame_timestamp) {
        Stack<OdometrySnapshot> tempSnapshots = new Stack<OdometrySnapshot>();
        tempSnapshots.addAll(odometrySnapshots);
        if(tempSnapshots.size() < 1) {
            odometry.resetPosition(
                Hardware.navX.getRotation2d(), 
                leftPosition.getValue() * Constants.Drive.rotorToMeters,
                rightPosition.getValue() * Constants.Drive.rotorToMeters,
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
                leftPosition.getValue() * Constants.Drive.rotorToMeters,
                rightPosition.getValue() * Constants.Drive.rotorToMeters,
                cameraPose);
                lastCameraCorrection = System.nanoTime();
            return;
        }

        odometry.resetPosition(
                closestSnapshot.gyroRotation, 
                closestSnapshot.leftDistance,
                closestSnapshot.rightDistance,
                cameraPose);
        int numFusedSnapshots = 0;
        for (int i = 0; i < tempSnapshots.size(); i++) {
            OdometrySnapshot snapshot = tempSnapshots.pop();
            odometry.update(
                snapshot.gyroRotation,
                snapshot.leftDistance, 
                snapshot.rightDistance
            );
            numFusedSnapshots++;
        }
        Subsystems.telemetry.pushDouble("tracking.numFusedSnapshots", numFusedSnapshots);
        lastCameraCorrection = System.nanoTime();
    }

    public void setOdometryPose(Pose2d pose) {
        odometry.resetPosition(getFieldRelativeRotation(), 
            leftPosition.getValue() * Constants.Drive.rotorToMeters, 
            rightPosition.getValue() * Constants.Drive.rotorToMeters,
            pose
        );
    }

    public ChassisSpeeds getChassisSpeeds() {
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
            leftVelocity.getValue()  * Constants.Drive.rotorToMeters,
            rightVelocity.getValue() * Constants.Drive.rotorToMeters
        );

        return Hardware.kinematics.toChassisSpeeds(wheelSpeeds);
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
        posePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getStructTopic("MyPose", Pose2d.struct).publish();

        initOdometry();
        client = new PoseStreamerClient("vision.local", 8833);
        client.start();
        client.requestPose(1, 1, (frame) -> {
            for(int i=0; i<frame.values.size(); i++) {
                Subsystems.telemetry.pushDouble("tracking.cameraPose" + i, frame.values.get(i));
            }
            Subsystems.telemetry.pushDouble("tracking.cameraPoseDeltaT", (System.nanoTime() - frame.timestamp)/1000000.0);
            if(frame.values.get(5) > 1 && frame.values.get(4) < 0.5) { //require at least 2 tags
                robotX = frame.values.get(0);
                robotY = frame.values.get(1);
                cameraPose = new Pose2d(robotX/1000.0, robotY/1000.0, new Rotation2d(frame.values.get(3) + Math.PI));
                synchronized(odometry) {
                    fuseCameraPose(cameraPose, frame.timestamp - 350000000);
                }
                

            }
        });

        client.requestPose(2, 4, (frame) -> {
            for(int i=0; i<frame.values.size(); i++) {
                Subsystems.telemetry.pushDouble("tracking.tagPose" + i, frame.values.get(i));
            }
            Subsystems.telemetry.pushDouble("tracking.tagPoseDeltaT", (System.nanoTime() - frame.timestamp)/1000000.0);
            shotTargetX = frame.values.get(0);
            shotTargetY = frame.values.get(1);
        });

        client.requestPose(3, 1, (frame) -> {
            Subsystems.telemetry.pushDouble("tracking.notePoseX", frame.values.get(0) -320);
            Subsystems.telemetry.pushDouble("tracking.notePoseY", frame.values.get(1) -240);
            noteTargetX = frame.values.get(0) - 320;
            noteTargetX = frame.values.get(1) - 240;
        });
        client.awaiting_clock_request = true;
    }

    public void publishTelemetry() {
        Subsystems.telemetry.pushDouble("tracking.odometryX", odometry.getPoseMeters().getX());
        Subsystems.telemetry.pushDouble("tracking.odometryY", odometry.getPoseMeters().getY());
        Subsystems.telemetry.pushDouble("tracking.odometryRadians", odometry.getPoseMeters().getRotation().getRadians());
        Subsystems.telemetry.pushDouble("tracking.leftPosition", leftPosition.getValue());
        Subsystems.telemetry.pushDouble("tracking.rightPosition", rightPosition.getValue());
        Subsystems.telemetry.pushDouble("tracking.theta", getFieldRelativeRotation().getDegrees());
        Subsystems.telemetry.pushDouble("tracking.omegaRadiansPerSecondGyro", Math.toRadians(Hardware.navX.getRate()));
    }

    public void onLoop(RunContext ctx) {
        BaseStatusSignal.refreshAll(leftPosition, rightPosition, rightVelocity, leftVelocity);

        synchronized(odometry) {
            updateOdometry();
        }
        posePublisher.set(odometry.getPoseMeters());
    }

    public void onStop() {}
}
