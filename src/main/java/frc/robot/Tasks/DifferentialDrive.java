package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

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


    
    public DifferentialDrive() {
        speedLimit = new Parameter<Double>(3.0);
    }

    public void onStart(RunContext context) {
        if(context == RunContext.teleoperated) {
            useStick();
        }
    }

    public void onStop() {
        coast();
    }

    public void onLoop(RunContext context) {
        /*if(Hardware.driverStick.getRawButtonPressed(1)) {
            calibrateFeedforward();
        }*/

        Subsystems.telemetry.pushString("DifferentialDriveMode", mode.toString());
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
    }

    public DriveMode getMode() {
        return mode;
    }

    public void useStick() {
        Subsystems.telemetry.pushEvent("DifferentialDrive.EnterStick");
        //setBrake(false);
        leftVelocityRequest = new VelocityVoltage(0);
        rightVelocityRequest = new VelocityVoltage(0);
        leftVelocityRequest.UpdateFreqHz = 60;
        rightVelocityRequest.UpdateFreqHz = 60;

        Hardware.leftDriveLeader.setControl(leftVelocityRequest);
        Hardware.rightDriveLeader.setControl(rightVelocityRequest);
        mode = DriveMode.legacy_stick;
    }

    void initVelocityDrive() {
        leftVelocityRequest = new VelocityVoltage(0);
        rightVelocityRequest = new VelocityVoltage(0);
        leftVelocityRequest.UpdateFreqHz = 63;
        rightVelocityRequest.UpdateFreqHz = 63;
        Hardware.leftDriveLeader.setControl(leftVelocityRequest);
        Hardware.rightDriveLeader.setControl(rightVelocityRequest);
        mode = DriveMode.modern_velocity;
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
        Subsystems.telemetry.pushDouble("DifferentialDrive.leftVelocityTarget", leftSpeed);
        Subsystems.telemetry.pushDouble("DifferentialDrive.rightVelocityTarget", rightSpeed);
        
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.autonomous);
            add(RunContext.teleoperated);
        }};
    }
}
