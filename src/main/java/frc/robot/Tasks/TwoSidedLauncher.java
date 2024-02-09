package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
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

    Timer shotTimer;

    public void launchNote() {
        Hardware.leftLauncherStage2.setControl(leftStage2Control);
        Hardware.rightLauncherStage2.setControl(rightStage2Control);
        //Hardware.leftLauncherStage1.setControl(stage1VelocityControl);
        //Hardware.rightLauncherStage1.setControl(stage1VelocityControl);

        Hardware.leftLauncherStage1.setControl(new CoastOut());
        Hardware.rightLauncherStage1.setControl(new CoastOut());
        stage2Active = true;
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
    }

    //returns whether the stage 2 rollers are within tolerances and ready to launch a note
    boolean stage2Ready() {
        return
            (Math.abs(leftStage2Control.Velocity - leftStage2VelocitySignal.getValue()) < Constants.Launcher.stage2Tolerance) &&
            (Math.abs(rightStage2Control.Velocity - rightStage2VelocitySignal.getValue()) < Constants.Launcher.stage2Tolerance);
    }

    public boolean finished() {
        return shotTimer.get() > 0.5;
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
        leftStage2VelocitySignal = Hardware.leftLauncherStage2.getVelocity();
        rightStage2VelocitySignal = Hardware.rightLauncherStage2.getVelocity();

        leftStage2VelocitySignal.setUpdateFrequency(66);
        rightStage2VelocitySignal.setUpdateFrequency(66);

        stage1Control = new VoltageOut(Constants.Launcher.stage1Voltage);
        stage1VelocityControl= new VelocityVoltage(Constants.Launcher.stage1Velocity);

        leftStage2Control = new VelocityVoltage(0);
        rightStage2Control = new VelocityVoltage(0);

        leftStage2Control.UpdateFreqHz = 66;
        rightStage2Control.UpdateFreqHz = 66;

        linearVelocity = new Parameter<Double>(Constants.Launcher.defaultVelocity);
        spinVelocity = new Parameter<Double>(Constants.Launcher.defaultSpinVelocity);

        velocityParametersUpdated();

        linearVelocity.onValueUpdated = value -> velocityParametersUpdated();
        spinVelocity.onValueUpdated = value -> velocityParametersUpdated();

        stopLauncher();

        shotTimer = new Timer();
        shotTimer.stop();
        shotTimer.reset();

        tiltPositionControl = new PositionVoltage(Constants.Launcher.tiltDefaultPosition);
        tiltPositionControl.UpdateFreqHz = 66;
        tiltPositionSignal = Hardware.launcherTiltMotor.getPosition();
        Hardware.launcherTiltMotor.setControl(tiltPositionControl);
        tiltPositionTarget = Constants.Launcher.tiltDefaultPosition;
        lastTiltTargetTS = System.nanoTime();
    }

    public void onLoop(RunContext ctx) {
        BaseStatusSignal.refreshAll(leftStage2VelocitySignal, rightStage2VelocitySignal, tiltPositionSignal);

        if(Hardware.operatorStick.getRawButtonPressed(Constants.OperatorControls.launcherButton)) {
            launchNote();
        } else if (Hardware.operatorStick.getRawButtonReleased(Constants.OperatorControls.launcherButton)) {
            stopLauncher();
        }

        Subsystems.telemetry.pushBoolean("launcher.stage2Ready", stage2Ready());

        if(stage2Ready() && !stage1Active && stage2Active) {
            stage1Active = true;
            Subsystems.intake.feedLauncher();
            shotTimer.reset();
            shotTimer.start();
            Hardware.leftLauncherStage1.setControl(stage1Control);
            Hardware.rightLauncherStage1.setControl(stage1Control);
        }

        /*if(Hardware.driverStick.getRawButtonPressed(1)) {
            Hardware.launcherTiltMotor.setControl(new VoltageOut(0.2));
        } else if (Hardware.driverStick.getRawButtonPressed(4)) {
            Hardware.launcherTiltMotor.setControl(new VoltageOut(-0.2));
        }

        if(
            Hardware.driverStick.getRawButtonReleased(1) ||
            Hardware.driverStick.getRawButtonReleased(4)
        ) {
            Hardware.launcherTiltMotor.setControl(new StaticBrake());
        }*/

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

        tiltPositionTarget = (tiltPositionTarget > Constants.Launcher.tiltMaxPosition)? Constants.Launcher.tiltMaxPosition : tiltPositionTarget;
        tiltPositionTarget = (tiltPositionTarget < Constants.Launcher.tiltMinPosition)? Constants.Launcher.tiltMinPosition : tiltPositionTarget;

        lastTiltTargetTS = System.nanoTime();

        Subsystems.telemetry.pushDouble("launcher.tiltPositionTarget", tiltPositionTarget);

        if(Hardware.driverStick.getRawButtonPressed(1)) {
            tiltPositionTarget = 0;
        } else if (Hardware.driverStick.getRawButtonPressed(4)) {
            tiltPositionTarget = 0.0874;
        }

        tiltPositionControl.Position = tiltPositionTarget;
        Hardware.launcherTiltMotor.setControl(tiltPositionControl);

        //note exit counter is connected to a prox sensor at the end of the shooter
        //exit counter will reach 2 once a note has fully cleared the sensor(2 rising edges on the signal)
        Subsystems.telemetry.pushDouble("launcher.noteExitCounter", Hardware.noteExitCounter.get());
    
        Subsystems.telemetry.pushDouble("launcher.leftStage2Velocity", leftStage2VelocitySignal.getValue());
        Subsystems.telemetry.pushDouble("launcher.rightStage2Velocity", rightStage2VelocitySignal.getValue());
        Subsystems.telemetry.pushBoolean("launcher.stage2Ready", stage2Ready());
        Subsystems.telemetry.pushDouble("launcher.tilt.position", tiltPositionSignal.getValue());
        Subsystems.telemetry.pushDouble("launcher.applied", Hardware.launcherTiltMotor.getMotorVoltage().getValue());
    }

    public void onStop() {
        stopLauncher();
    }
}
