package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;


import edu.wpi.first.wpilibj.Timer;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.Parameter;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Subsystems;

public class TwoSidedLauncher implements IPeriodicTask {
    //TODO: calibrate launcher PIDs
    VelocityVoltage leftStage2Control;
    VelocityVoltage rightStage2Control;
    VelocityVoltage stage1VelocityControl;
    PositionVoltage tiltPositionControl;
    //left and right stage 1 motors share a control object
    VoltageOut stage1Control;

    StatusSignal<Double> leftStage2VelocitySignal;
    StatusSignal<Double> rightStage2VelocitySignal;

    StatusSignal<Double> tiltPositionSignal;

    boolean stage1Active;
    boolean stage2Active;

    double tiltPositionTarget = 0;
    long lastTiltTargetTS;

    public Parameter<Double> linearVelocity;
    //spin velocity controls the difference between left and right velocities to control the direction of the note
    //positive values curve the note to the right, negative curves to the left
    public Parameter<Double> spinVelocity;

    public Parameter<Double> tilt;

    Timer shotTimer;

    public TwoSidedLauncher() {
        leftStage2VelocitySignal = Hardware.leftLauncherStage2.getVelocity();
        rightStage2VelocitySignal = Hardware.rightLauncherStage2.getVelocity();
        leftStage2VelocitySignal.setUpdateFrequency(200);
        rightStage2VelocitySignal.setUpdateFrequency(200);
        leftStage2Control = new VelocityVoltage(0);
        rightStage2Control = new VelocityVoltage(0);
        stage1VelocityControl= new VelocityVoltage(Constants.Launcher.stage1Velocity);

        linearVelocity = new Parameter<Double>(Constants.Launcher.defaultVelocity);
        spinVelocity = new Parameter<Double>(Constants.Launcher.defaultSpinVelocity);
        linearVelocity.onValueUpdated = value -> velocityParametersUpdated();
        spinVelocity.onValueUpdated = value -> velocityParametersUpdated();

        tilt = new Parameter<Double>(Constants.Launcher.tiltDefaultPosition);
        tilt.onValueUpdated = position -> setTiltPosition(position);
    }

    public void launchNote() {
        if(stage2Active) return;
        Hardware.leftLauncherStage2.setControl(leftStage2Control);
        Hardware.rightLauncherStage2.setControl(rightStage2Control);
        Hardware.leftLauncherStage1.setControl(leftStage2Control);
        Hardware.rightLauncherStage1.setControl(rightStage2Control);

        stage2Active = true;
        shotTimer.stop();
        shotTimer.reset();
    }

    public void stopLauncher() {
        Hardware.leftLauncherStage1.setControl(new CoastOut());
        Hardware.rightLauncherStage1.setControl(new CoastOut());
        Hardware.leftLauncherStage2.setControl(new CoastOut());
        Hardware.rightLauncherStage2.setControl(new CoastOut());
        stage1Active = false;
        stage2Active = false;
        Subsystems.intake.cancelFeed();
        Subsystems.intake.clearNote();
        shotTimer.stop();
        shotTimer.reset();
    }

    //returns whether the stage 2 rollers are within tolerances and ready to launch a note
    boolean stage2Ready() {
        return
            (Math.abs(leftStage2Control.Velocity - leftStage2VelocitySignal.getValue()) < Constants.Launcher.stage2Tolerance) &&
            (Math.abs(rightStage2Control.Velocity - rightStage2VelocitySignal.getValue()) < Constants.Launcher.stage2Tolerance);
    }

    public boolean finished() {
        return shotTimer.get() > 0.35;
    }

    public void setTiltPosition(double target) {
        target = (target > Constants.Launcher.tiltMaxPosition)? Constants.Launcher.tiltMaxPosition : target;
        target = (target < Constants.Launcher.tiltMinPosition)? Constants.Launcher.tiltMinPosition : target;
        tiltPositionControl.Position = target;
        Hardware.launcherTiltMotor.setControl(tiltPositionControl);
    }

    public boolean tiltFinished() {
        return Math.abs(tiltPositionControl.Position - tiltPositionSignal.getValue()) < Constants.Launcher.tiltTolerance;
    }

    void velocityParametersUpdated() {
        leftStage2Control.Velocity = (linearVelocity.getValue() + spinVelocity.getValue()); //TODO: roto to meters
        rightStage2Control.Velocity = (linearVelocity.getValue() - spinVelocity.getValue());
        Subsystems.telemetry.pushDouble("launcher.linearVelocity", linearVelocity.getValue());
        Subsystems.telemetry.pushDouble("launcher.spinVelocity", spinVelocity.getValue());
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.teleoperated);
            add(RunContext.autonomous);
        }};
    }

    public void onStart(RunContext ctx) {


        stage1Control = new VoltageOut(Constants.Launcher.stage1Voltage);
        

        leftStage2Control.Velocity = 0;
        rightStage2Control.Velocity = 0;

        leftStage2Control.UpdateFreqHz = 200;
        rightStage2Control.UpdateFreqHz = 200;

        velocityParametersUpdated();

        shotTimer = new Timer();

        stopLauncher();

        
        shotTimer.stop();
        shotTimer.reset();

        tiltPositionControl = new PositionVoltage(Constants.Launcher.tiltDefaultPosition);
        tiltPositionControl.UpdateFreqHz = 200;
        tiltPositionSignal = Hardware.launcherTiltMotor.getPosition();
        Hardware.launcherTiltMotor.setControl(tiltPositionControl);
        tiltPositionTarget = Constants.Launcher.tiltDefaultPosition;
        lastTiltTargetTS = System.nanoTime();
    }

    public void onLoop(RunContext ctx) {
        BaseStatusSignal.refreshAll(leftStage2VelocitySignal, rightStage2VelocitySignal, tiltPositionSignal);

        Subsystems.telemetry.pushBoolean("launcher.stage2Ready", stage2Ready());

        if(stage2Ready() && !stage1Active && stage2Active) {
            stage1Active = true;
            Subsystems.intake.feedLauncher();
        }

        if(stage1Active && stage2Active && !Subsystems.intake.getIndexSensor() && shotTimer.get() ==0) {
            shotTimer.reset();
            shotTimer.start();
        }

        double tiltDeltaT = (System.nanoTime() - lastTiltTargetTS)/1000000000.0;

        double x = -Hardware.driverStick.getRawAxis(5);
        x = (Math.abs(x) > Constants.Drive.deadZone)? 
        ((x > 0)? 
            ((x-Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) :
            ((x+Constants.Drive.deadZone)/(1-Constants.Drive.deadZone))
        ) 
        : 0;

        double toMove = tiltDeltaT * x * 0.05;

        tiltPositionTarget += toMove;

        lastTiltTargetTS = System.nanoTime();

        Subsystems.telemetry.pushDouble("launcher.tiltPositionTarget", tiltPositionTarget);

        //note exit counter is connected to a prox sensor at the end of the shooter
        //exit counter will reach 2 once a note has fully cleared the sensor(2 rising edges on the signal)
    
        Subsystems.telemetry.pushDouble("launcher.noteExitCounter", Hardware.noteExitCounter.get());
    
        Subsystems.telemetry.pushDouble("launcher.leftStage2Velocity", leftStage2VelocitySignal.getValue());
        Subsystems.telemetry.pushDouble("launcher.rightStage2Velocity", rightStage2VelocitySignal.getValue());
        Subsystems.telemetry.pushBoolean("launcher.stage2Ready", stage2Ready());
        Subsystems.telemetry.pushBoolean("launcher.stage1Ready", stage1Active);
        Subsystems.telemetry.pushDouble("launcher.tilt.position", tiltPositionSignal.getValue());
        Subsystems.telemetry.pushDouble("launcher.applied", Hardware.launcherTiltMotor.getMotorVoltage().getValue());
        Subsystems.telemetry.pushBoolean("launcher.tilt.finished", tiltFinished());
    }

    public void publishTelemetry() {
    }

    public void onStop() {
        stopLauncher();
    }
}
