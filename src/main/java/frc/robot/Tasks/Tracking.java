package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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

    StatusSignal<Double> leftVelocity;
    StatusSignal<Double> rightVelocity;

    PoseStreamerClient client;

    public double shotTargetX;
    public double shotTargetY;

    public double robotX;
    public double robotY;

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
    }

    void updateOdometry() {
        odometry.update(
            getFieldRelativeRotation(),
            leftPosition.getValue() * Constants.Drive.rotorToMeters, 
            rightPosition.getValue() * Constants.Drive.rotorToMeters
        );
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
        initOdometry();
        client = new PoseStreamerClient("frc-6978-vision.local", 8833);
        client.start();
        client.requestPose(1, 1, (values) -> {
            for(int i=0; i<values.size(); i++) {
                Subsystems.telemetry.pushDouble("tracking.cameraPose" + i, values.get(i));
            }
            robotX = values.get(0);
            robotY = values.get(1);
        });

        client.requestPose(2, 4, (values) -> {
            for(int i=0; i<values.size(); i++) {
                Subsystems.telemetry.pushDouble("tracking.tagPose" + i, values.get(i));
            }
            shotTargetX = values.get(0);
            shotTargetY = values.get(1);
        });
        client.awaiting_clock_request = true;
    }

    public void publishTelemetry() {
        Subsystems.telemetry.pushDouble("tracking.odometryX", odometry.getPoseMeters().getX());
        Subsystems.telemetry.pushDouble("tracking.odometryY", odometry.getPoseMeters().getY());
        Subsystems.telemetry.pushDouble("tracking.leftPosition", leftPosition.getValue());
        Subsystems.telemetry.pushDouble("tracking.rightPosition", rightPosition.getValue());
        Subsystems.telemetry.pushDouble("tracking.theta", getFieldRelativeRotation().getDegrees());
        Subsystems.telemetry.pushDouble("tracking.omegaRadiansPerSecondGyro", Math.toRadians(Hardware.navX.getRate()));
    }

    public void onLoop(RunContext ctx) {
        BaseStatusSignal.refreshAll(leftPosition, rightPosition, rightVelocity, leftVelocity);

        updateOdometry();
    }

    public void onStop() {}
}
