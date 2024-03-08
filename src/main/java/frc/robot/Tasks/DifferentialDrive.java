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

    RunContext lastContext;
    
    public DifferentialDrive() {
        speedLimit = new Parameter<Double>(3.0);
    }

    public void onStart(RunContext context) {
        if(context == RunContext.teleoperated) {
            //useStick();
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
            case legacy_stick:
                driveStickVelocity();
                break;
            case calibrate:
                onCalibrateFeedforward();
                break;
            default:
                break;
        }
        Subsystems.telemetry.pushDouble("leftApplied", Hardware.leftDriveLeader.getMotorVoltage().getValue());
        Subsystems.telemetry.pushDouble("rightApplied", Hardware.rightDriveLeader.getMotorVoltage().getValue());

    }

    public DriveMode getMode() {
        return mode;
    }

    public void useStick() {
        Subsystems.telemetry.pushEvent("DifferentialDrive.EnterStick");
        //setBrake(false);
        leftVelocityRequest = new VelocityVoltage(0);
        rightVelocityRequest = new VelocityVoltage(0);
        leftVelocityRequest.UpdateFreqHz = 200;
        rightVelocityRequest.UpdateFreqHz = 200;

        Hardware.leftDriveLeader.setControl(leftVelocityRequest);
        Hardware.rightDriveLeader.setControl(rightVelocityRequest);
        mode = DriveMode.legacy_stick;
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

    void driveStickVelocity() {
        double x;
        double y;

        x = Hardware.driverStick.getRawAxis(Constants.DriverControls.steeringAxis);
        y = (Hardware.driverStick.getRawAxis(Constants.DriverControls.forwardAxis) + 1 )/2 - 
            (Hardware.driverStick.getRawAxis(Constants.DriverControls.reverseAxis) + 1 )/2;

        x = (Math.abs(x) > Constants.Drive.deadZone)? 
            ((x > 0)? 
                ((x-Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) :
                ((x+Constants.Drive.deadZone)/(1-Constants.Drive.deadZone))
            ) 
            : 0;

        y = (Math.abs(y) > Constants.Drive.deadZone)? 
            ((y > 0)? 
                ((y-Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) :
                ((y+Constants.Drive.deadZone)/(1-Constants.Drive.deadZone))
            ) 
            : 0;

        x *= Constants.Drive.xMultiplier;
        y *= Constants.Drive.yMultiplier;

        Subsystems.telemetry.pushDouble("DriveX", x);
        Subsystems.telemetry.pushDouble("DriveY", y);

        driveVelocity(x, y);
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

        Subsystems.telemetry.pushDouble("leftFeedForwardRatio", input / Hardware.leftDriveLeader.getVelocity().getValue());
        Subsystems.telemetry.pushDouble("rightFeedForwardRatio", input / Hardware.rightDriveLeader.getVelocity().getValue());

        Subsystems.telemetry.pushDouble("calibrationRequestedVoltage", input);
        calibrationOut.Output = input;
        Hardware.leftDriveLeader.setControl(calibrationOut);
        Hardware.rightDriveLeader.setControl(calibrationOut);
    }

    public void driveChassisSpeeds(ChassisSpeeds speeds) {
        if(mode != DriveMode.modern_velocity) {
            initVelocityDrive();
        }

        Subsystems.telemetry.pushDouble("drive.requestedOmega", speeds.omegaRadiansPerSecond);
        Subsystems.telemetry.pushDouble("drive.requestedVX", speeds.vxMetersPerSecond);


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
        Subsystems.telemetry.pushDouble("DifferentialDrive.leftVelocityTarget", left);
        Subsystems.telemetry.pushDouble("DifferentialDrive.rightVelocityTarget", right);
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

    void driveVelocity(double angular, double linear) {
        if(linear > Constants.Drive.maxLinearVelocity){
            linear = Constants.Drive.maxLinearVelocity;
        }else if(linear < -Constants.Drive.maxLinearVelocity){
            linear = -Constants.Drive.maxLinearVelocity;
        }

        if(angular > Constants.Drive.maxAngularVelocity){
            angular = Constants.Drive.maxAngularVelocity;
        }else if(angular < -Constants.Drive.maxAngularVelocity){
            angular = -Constants.Drive.maxAngularVelocity;
        }
        
        double leftSpeed = (linear + angular)/Constants.Drive.rotorToMeters;
        double rightSpeed = (linear - angular)/Constants.Drive.rotorToMeters;

        leftVelocityRequest.Velocity = leftSpeed;
        rightVelocityRequest.Velocity = rightSpeed;
        Hardware.leftDriveLeader.setControl(leftVelocityRequest);
        Hardware.rightDriveLeader.setControl(rightVelocityRequest);
        Subsystems.telemetry.pushDouble("drive.leftVelocityTarget", leftSpeed);
        Subsystems.telemetry.pushDouble("drive.rightVelocityTarget", rightSpeed);
        
    }

    public void publishTelemetry() {
        Subsystems.telemetry.pushDouble("drive.leftLeader.motorVoltage", Hardware.leftDriveLeader.getMotorVoltage().getValue());
        Subsystems.telemetry.pushDouble("drive.rightLeader.motorVoltage", Hardware.rightDriveLeader.getMotorVoltage().getValue());

        Subsystems.telemetry.pushDouble("drive.leftLeader.temperature", Hardware.leftDriveLeader.getDeviceTemp().getValue());
        Subsystems.telemetry.pushDouble("drive.leftLeader.current", Hardware.leftDriveLeader.getStatorCurrent().getValue());
        Subsystems.telemetry.pushDouble("drive.leftFollower.temperature", Hardware.leftDrive2.getDeviceTemp().getValue());
        Subsystems.telemetry.pushDouble("drive.leftFollower.current", Hardware.leftDrive2.getStatorCurrent().getValue());


        Subsystems.telemetry.pushDouble("drive.rightLeader.temperature", Hardware.rightDriveLeader.getDeviceTemp().getValue());
        Subsystems.telemetry.pushDouble("drive.rightLeader.current", Hardware.rightDriveLeader.getStatorCurrent().getValue());
        Subsystems.telemetry.pushDouble("drive.rightFollower.temperature", Hardware.rightDrive2.getDeviceTemp().getValue());
        Subsystems.telemetry.pushDouble("drive.rightFollower.current", Hardware.rightDrive2.getStatorCurrent().getValue());
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.autonomous);
            add(RunContext.teleoperated);
        }};
    }
}
