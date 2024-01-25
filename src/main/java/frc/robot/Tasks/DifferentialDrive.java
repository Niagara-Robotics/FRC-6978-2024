package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.PIDController;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Tasks;

public class DifferentialDrive implements IPeriodicTask{
    enum DriveMode {
        stick,
        balance,
        heading,
        distance,
        calibrate,
    };
    DriveMode mode;

    VoltageOut calibrationOut;

    VelocityVoltage leftVelocityRequest;
    VelocityVoltage rightVelocityRequest;

    StatusSignal<Double> leftPosition;
    StatusSignal<Double> rightPosition;

    DifferentialDriveOdometry odometry;

    PIDController balancingYawController = new PIDController(
        Constants.Drive.balancer_yaw_kP,
        Constants.Drive.balancer_yaw_kI,
        Constants.Drive.balancer_yaw_kD
    );
    PIDController balancingPitchController = new PIDController(
        Constants.Drive.balancer_pitch_kP,
        Constants.Drive.balancer_pitch_kI,
        Constants.Drive.balancer_pitch_kD
    );

    boolean gearShiftState;
    Timer calibrationTimer;
    
    public DifferentialDrive() {
        
    }

    void initOdometry() {
        odometry = new DifferentialDriveOdometry(
            Hardware.navX.getRotation2d(),
            leftPosition.getValue() * Constants.Drive.wheelRotorRatio, 
            rightPosition.getValue() * Constants.Drive.wheelRotorRatio,
            new Pose2d(0, 0, new Rotation2d())
        );
    }

    void updateOdometry() {
        Pose2d pose = odometry.update(
            Hardware.navX.getRotation2d(),
            leftPosition.getValue() * Constants.Drive.wheelRotorRatio, 
            rightPosition.getValue() * Constants.Drive.wheelRotorRatio
        );

        Tasks.telemetry.pushDouble("odometryX", pose.getX());
        Tasks.telemetry.pushDouble("odometryY", pose.getY());
        Tasks.telemetry.pushDouble("leftPosition", leftPosition.getValue());
        Tasks.telemetry.pushDouble("rightPosition", rightPosition.getValue());
    }

    public void onStart(RunContext context) {
        gearShiftState = Constants.Drive.gearShiftDefaultState;
   
        //Hardware.driveGearShiftSolenoid.set(gearShiftState);
        setBrake(false);
        if(context == RunContext.teleoperated) {
            useStick();
        }
        leftPosition = Hardware.leftDrive1.getPosition();
        rightPosition = Hardware.rightDrive1.getPosition();

        leftPosition.setUpdateFrequency(63);
        rightPosition.setUpdateFrequency(63);

        initOdometry();
    }

    public void onStop() {}

    public void onLoop(RunContext context) {
        BaseStatusSignal.refreshAll(leftPosition, rightPosition);

        updateOdometry();

        if(Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.balance) || 
            Hardware.driverStick.getRawButtonReleased(1)) {
            useStick();
        }

        if(Hardware.driverStick.getRawButtonPressed(1)) {
            calibrationOut = new VoltageOut(0);
            calibrationOut.UpdateFreqHz = 50;
            Hardware.leftDrive1.setControl(calibrationOut);
            Hardware.rightDrive1.setControl(calibrationOut);
            calibrationTimer = new Timer();
            calibrationTimer.start();
            mode = DriveMode.calibrate;
        }

        Tasks.telemetry.pushDouble("theta", Hardware.navX.getYaw());

        Tasks.telemetry.pushString("DifferentialDriveMode", mode.toString());
        switch (mode) {
            case stick:
                onStickDrive();
                break;
            case balance:
                onBalance();
                break;
            case calibrate:
                onCalibrateFeedforward();
                break;
            default:
                break;
        }
    }

    public void useStick() {
        Tasks.telemetry.pushEvent("DifferentialDrive.EnterStick");
        //setBrake(false);
        leftVelocityRequest = new VelocityVoltage(0);
        rightVelocityRequest = new VelocityVoltage(0);
        leftVelocityRequest.UpdateFreqHz = 60;
        rightVelocityRequest.UpdateFreqHz = 60;

        Hardware.leftDrive1.setControl(leftVelocityRequest);
        Hardware.rightDrive1.setControl(rightVelocityRequest);
        mode = DriveMode.stick;
    }

    void onStickDrive() {
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

        Tasks.telemetry.pushDouble("DriveX", x);
        Tasks.telemetry.pushDouble("DriveY", y);

        setDrives(x, y);        
    }

    void onBalance() {
        setDrives(
            balancingYawController.process(Hardware.navX.getAngle()), 
            balancingPitchController.process(Hardware.navX.getPitch())
            );
    }

    void onCalibrateFeedforward() {
        if(calibrationTimer.get() > 10) {
            mode = DriveMode.stick;
        }
        Tasks.telemetry.pushDouble("calibrationTime", calibrationTimer.get());
        double input = ((int)((calibrationTimer.get() / 10.0) * 10));

        Tasks.telemetry.pushDouble("leftFeedForwardRatio", input / Hardware.leftDrive1.getVelocity().getValue());
        Tasks.telemetry.pushDouble("rightFeedForwardRatio", input / Hardware.rightDrive1.getVelocity().getValue());

        Tasks.telemetry.pushDouble("calibrationRequestedVoltage", input);
        calibrationOut.Output = input;
        Hardware.leftDrive1.setControl(calibrationOut);
        Hardware.rightDrive1.setControl(calibrationOut);
    }

    void setDrives(double x, double y) {
        if(y > Constants.Drive.maxY){
            y = Constants.Drive.maxY;
        }else if(y < -Constants.Drive.maxY){
            y = -Constants.Drive.maxY;
        }

        if(x > Constants.Drive.maxY){
            x = Constants.Drive.maxY;
        }else if(x < -Constants.Drive.maxY){
            x = -Constants.Drive.maxY;
        }
        
        double leftSpeed = (y + x)*8/*Constants.Drive.maxVelocity*/;
        double rightSpeed = (y - x)*8/*Constants.Drive.maxVelocity*/;

        if(x == 0 && y == 0) {
            Hardware.leftDrive1.setControl(new NeutralOut());
            Hardware.rightDrive1.setControl(new NeutralOut());
        } else {
            leftVelocityRequest.Velocity = leftSpeed;
            rightVelocityRequest.Velocity = rightSpeed;
            Hardware.leftDrive1.setControl(leftVelocityRequest);
            Hardware.rightDrive1.setControl(rightVelocityRequest);
            Tasks.telemetry.pushDouble("DifferentialDrive.leftVelocityTarget", leftSpeed);
            Tasks.telemetry.pushDouble("DifferentialDrive.rightVelocityTarget", rightSpeed);
        }
    }

    //TODO: replace this with control modes, blocking config request takes over 250ms
    void setBrake(boolean brake) {
        Tasks.telemetry.pushEvent("DifferentialDrive.SetBrakeMode");
        Tasks.telemetry.pushBoolean("DifferentialDrive.BrakeMode", brake);
        NeutralModeValue neutralMode = brake? NeutralModeValue.Brake : NeutralModeValue.Coast; 
        //config requests in question
        Hardware.leftDrive1.setNeutralMode(neutralMode);
        Hardware.leftDrive2.setNeutralMode(neutralMode);
        Hardware.rightDrive1.setNeutralMode(neutralMode);
        Hardware.rightDrive2.setNeutralMode(neutralMode);
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.autonomous);
            add(RunContext.teleoperated);
        }};
    }
}
