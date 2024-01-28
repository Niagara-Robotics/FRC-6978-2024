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

    void initOdometry() {
        leftPosition = Hardware.leftDriveLeader.getPosition();
        rightPosition = Hardware.rightDriveLeader.getPosition();

        //change the update frequency depending on the loop time
        leftPosition.setUpdateFrequency(63);
        rightPosition.setUpdateFrequency(63);

        leftVelocity = Hardware.leftDriveLeader.getVelocity();
        rightVelocity = Hardware.rightDriveLeader.getVelocity();

        leftVelocity.setUpdateFrequency(63);
        rightVelocity.setUpdateFrequency(63);

        odometry = new DifferentialDriveOdometry(
            Hardware.navX.getRotation2d(),
            leftPosition.getValue() * Constants.Drive.rotorToMeters,
            rightPosition.getValue() * Constants.Drive.rotorToMeters,
            new Pose2d(0, 0, new Rotation2d())
        );
    }

    void updateOdometry() {
        Pose2d pose = odometry.update(
            getFieldRelativeRotation(),
            leftPosition.getValue() * Constants.Drive.rotorToMeters, 
            rightPosition.getValue() * Constants.Drive.rotorToMeters
        );

        Subsystems.telemetry.pushDouble("odometryX", pose.getX());
        Subsystems.telemetry.pushDouble("odometryY", pose.getY());
        Subsystems.telemetry.pushDouble("leftPosition", leftPosition.getValue());
        Subsystems.telemetry.pushDouble("rightPosition", rightPosition.getValue());
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
            leftVelocity.getValue(),
            rightVelocity.getValue()
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
    }

    public void onLoop(RunContext ctx) {
        BaseStatusSignal.refreshAll(leftPosition, rightPosition);

        updateOdometry();
        Subsystems.telemetry.pushDouble("theta", getFieldRelativeRotation().getDegrees());
    }

    public void onStop() {}
}
