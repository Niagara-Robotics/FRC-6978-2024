package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

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
    //left and right stage 1 motors share a control object
    VoltageOut stage1Control;

    StatusSignal<Double> leftStage2VelocitySignal;
    StatusSignal<Double> rightStage2VelocitySignal;

    boolean stage1Active;
    boolean stage2Active;

    public Parameter<Double> linearVelocity;
    //spin velocity controls the difference between left and right velocities to control the direction of the note
    //positive values curve the note to the right, negative curves to the left
    public Parameter<Double> spinVelocity;

    public void launchNote() {
        Hardware.leftLauncherStage2.setControl(leftStage2Control);
        Hardware.rightLauncherStage2.setControl(rightStage2Control);

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
    }

    //returns whether the stage 2 rollers are within tolerances and ready to launch a note
    boolean stage2Ready() {
        return
            (Math.abs(linearVelocity.getValue() - leftStage2VelocitySignal.getValue()) < Constants.Launcher.stage2Tolerance) &&
            (Math.abs(linearVelocity.getValue() - rightStage2VelocitySignal.getValue()) < Constants.Launcher.stage2Tolerance);
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

        leftStage2VelocitySignal.setUpdateFrequency(63);
        rightStage2VelocitySignal.setUpdateFrequency(63);

        stage1Control = new VoltageOut(Constants.Launcher.stage1Voltage);

        leftStage2Control = new VelocityVoltage(0);
        rightStage2Control = new VelocityVoltage(0);

        leftStage2Control.UpdateFreqHz = 63;
        rightStage2Control.UpdateFreqHz = 63;

        linearVelocity = new Parameter<Double>(Constants.Launcher.defaultVelocity);
        spinVelocity = new Parameter<Double>(0.0);

        velocityParametersUpdated();

        linearVelocity.onValueUpdated = value -> velocityParametersUpdated();
        spinVelocity.onValueUpdated = value -> velocityParametersUpdated();

        stopLauncher();
    }

    public void onLoop(RunContext ctx) {
        BaseStatusSignal.refreshAll(leftStage2VelocitySignal, rightStage2VelocitySignal);

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.launcherButton)) {
            launchNote();
        } else if (Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.launcherButton)) {
            stopLauncher();
            Subsystems.intake.cancelFeed();
        }

        Subsystems.telemetry.pushBoolean("launcher.stage2Ready", stage2Ready());

        if(stage2Ready() && !stage1Active && stage2Active) {
            stage1Active = true;
            Subsystems.intake.feedLauncher();
            Hardware.leftLauncherStage1.setControl(stage1Control);
            Hardware.rightLauncherStage1.setControl(stage1Control);
        }
        //note exit counter is connected to a prox sensor at the end of the shooter
        //exit counter will reach 2 once a note has fully cleared the sensor(2 rising edges on the signal)
        Subsystems.telemetry.pushDouble("launcher.noteExitCounter", Hardware.noteExitCounter.get());
    
        Subsystems.telemetry.pushDouble("launcher.leftStage2Velocity", leftStage2VelocitySignal.getValue());
        Subsystems.telemetry.pushDouble("launcher.rightStage2Velocity", rightStage2VelocitySignal.getValue());
    }

    public void onStop() {}
}
