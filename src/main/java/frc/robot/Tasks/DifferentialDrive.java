package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Framework.IPeriodicTask;

import frc.robot.Framework.Parameter;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Subsystems;


public class DifferentialDrive implements IPeriodicTask{
    enum DriveMode {
        legacy_stick, //TODO: Remove legacy stick mode
        modern_velocity,
        calibrate,
        none,
    };
    DriveMode mode;

    VoltageOut calibrationOut;
    Timer calibrationTimer;

    VelocityVoltage leftVelocityRequest;
    VelocityVoltage rightVelocityRequest;

    Parameter<Double> speedLimit;

    Parameter<ChassisSpeeds> wantedChassisSpeeds;

    RunContext lastContext;
    
    public DifferentialDrive() {
        speedLimit = new Parameter<Double>(3.0);
        wantedChassisSpeeds = new Parameter<ChassisSpeeds>(new ChassisSpeeds());

        wantedChassisSpeeds.onValueUpdated = (speeds)->driveChassisSpeeds(speeds);
    }

    public void onStart(RunContext context) {
        if(context == RunContext.teleoperated) {
            mode = DriveMode.none;
        } else if(context == RunContext.autonomous) {
            Hardware.leftDriveLeader.setNeutralMode(NeutralModeValue.Brake);
            Hardware.rightDriveLeader.setNeutralMode(NeutralModeValue.Brake);
        }
        lastContext = context;
    }

    public void onStop() {
        if(lastContext == RunContext.autonomous) brake(); 
        else coast();
    }

    public void onLoop(RunContext context) {
        /*if(Hardware.driverStick.getRawButtonPressed(1)) {
            calibrateFeedforward();
        }*/

        switch (mode) {
            case calibrate:
                onCalibrateFeedforward();
                break;
            default:
                break;
        }
    }

    public DriveMode getMode() {
        return mode;
    }

    void initVelocityDrive() {
        leftVelocityRequest = new VelocityVoltage(0);
        rightVelocityRequest = new VelocityVoltage(0);
        leftVelocityRequest.UpdateFreqHz = 200;
        rightVelocityRequest.UpdateFreqHz = 200;
        Hardware.leftDriveLeader.setControl(leftVelocityRequest);
        Hardware.rightDriveLeader.setControl(rightVelocityRequest);
        mode = DriveMode.modern_velocity;
        System.out.println("DifferentialDrive: switched to modern velocity");
    }

    void calibrateFeedforward() {
        calibrationOut = new VoltageOut(0);
        calibrationOut.UpdateFreqHz = 50;
        Hardware.leftDriveLeader.setControl(calibrationOut);
        Hardware.rightDriveLeader.setControl(calibrationOut);
        calibrationTimer = new Timer();
        calibrationTimer.start();
        mode = DriveMode.calibrate;
    }

    void onCalibrateFeedforward() {
        if(calibrationTimer.get() > 10) {
            mode = DriveMode.legacy_stick;
        }
        Subsystems.telemetry.pushDouble("calibrationTime", calibrationTimer.get());
        double input = ((int)((calibrationTimer.get() / 10.0) * 10));

        Subsystems.telemetry.pushDouble("drive_leftFeedForwardRatio", input / Hardware.leftDriveLeader.getVelocity().getValue());
        Subsystems.telemetry.pushDouble("drive_rightFeedForwardRatio", input / Hardware.rightDriveLeader.getVelocity().getValue());

        Subsystems.telemetry.pushDouble("calibrationRequestedVoltage", input);
        calibrationOut.Output = input;
        Hardware.leftDriveLeader.setControl(calibrationOut);
        Hardware.rightDriveLeader.setControl(calibrationOut);
    }

    private void driveChassisSpeeds(ChassisSpeeds speeds) {
        if(mode != DriveMode.modern_velocity) {
            initVelocityDrive();
        }

        Subsystems.telemetry.pushDouble("drive_requestedOmega", speeds.omegaRadiansPerSecond);
        Subsystems.telemetry.pushDouble("drive_requestedVX", speeds.vxMetersPerSecond);


        DifferentialDriveWheelSpeeds wheelSpeeds = Hardware.kinematics.toWheelSpeeds(speeds);
        
        double left = wheelSpeeds.leftMetersPerSecond;
        double right = wheelSpeeds.rightMetersPerSecond;

        if(left > Constants.Drive.maxLinearVelocity) {
            left = Constants.Drive.maxLinearVelocity;
        } else if (left < -Constants.Drive.maxLinearVelocity) {
            left = -Constants.Drive.maxLinearVelocity;
        }

        if(right > Constants.Drive.maxLinearVelocity) {
            right = Constants.Drive.maxLinearVelocity;
        } else if (right < -Constants.Drive.maxLinearVelocity) {
            right = -Constants.Drive.maxLinearVelocity;
        }

        leftVelocityRequest.Velocity = left / Constants.Drive.rotorToMeters;
        rightVelocityRequest.Velocity = right / Constants.Drive.rotorToMeters;

        Hardware.leftDriveLeader.setControl(leftVelocityRequest);
        Hardware.rightDriveLeader.setControl(rightVelocityRequest);
        Subsystems.telemetry.pushDouble("drive_leftVelocityTarget", left);
        Subsystems.telemetry.pushDouble("drive_rightVelocityTarget", right);
    }

    public void coast() {
        Hardware.leftDriveLeader.setControl(new CoastOut());
        Hardware.rightDriveLeader.setControl(new CoastOut());
        mode = DriveMode.none;
    }

    public void brake() {
        Hardware.leftDriveLeader.setNeutralMode(NeutralModeValue.Brake);
        Hardware.rightDriveLeader.setNeutralMode(NeutralModeValue.Brake);
        Hardware.leftDriveLeader.setControl(new StaticBrake());
        Hardware.rightDriveLeader.setControl(new StaticBrake());
        mode = DriveMode.none;
    }

    public void publishTelemetry() {
        Subsystems.telemetry.pushDouble("drive_leftLeader_motorVoltage", Hardware.leftDriveLeader.getMotorVoltage().getValue());
        Subsystems.telemetry.pushDouble("drive_rightLeader_motorVoltage", Hardware.rightDriveLeader.getMotorVoltage().getValue());

        Subsystems.telemetry.pushDouble("drive_leftLeader_temperature", Hardware.leftDriveLeader.getDeviceTemp().getValue());
        Subsystems.telemetry.pushDouble("drive_leftLeader_current", Hardware.leftDriveLeader.getStatorCurrent().getValue());
        Subsystems.telemetry.pushDouble("drive_leftFollower_temperature", Hardware.leftDrive2.getDeviceTemp().getValue());
        Subsystems.telemetry.pushDouble("drive_leftFollower_current", Hardware.leftDrive2.getStatorCurrent().getValue());


        Subsystems.telemetry.pushDouble("drive_rightLeader_temperature", Hardware.rightDriveLeader.getDeviceTemp().getValue());
        Subsystems.telemetry.pushDouble("drive_rightLeader_current", Hardware.rightDriveLeader.getStatorCurrent().getValue());
        Subsystems.telemetry.pushDouble("drive_rightFollower_temperature", Hardware.rightDrive2.getDeviceTemp().getValue());
        Subsystems.telemetry.pushDouble("drive_rightFollower_current", Hardware.rightDrive2.getStatorCurrent().getValue());
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.autonomous);
            add(RunContext.teleoperated);
        }};
    }
}
