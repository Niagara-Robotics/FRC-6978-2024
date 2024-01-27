package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.Parameter;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Subsystems;

public class OneSidedLauncher implements IPeriodicTask {
    
    VelocityVoltage stage1Control;
    VelocityVoltage stage2Control;

    StatusSignal<Double> leftStage1VelocitySignal;
    StatusSignal<Double> leftStage2VelocitySignal;

    public Parameter<Double> linearVelocity;

    boolean stage1Active;
    boolean stage2Active;

    public void startLauncher() {
        //stage 2 spins up first
        Hardware.leftLauncherStage2.setControl(stage2Control);
        Hardware.leftLauncherStage1.setControl(new CoastOut());

        stage1Active = false;
        stage2Active = true;
    }

    public void stopLauncher() {
        Hardware.leftLauncherStage1.setControl(new CoastOut());
        Hardware.leftLauncherStage2.setControl(new CoastOut());
        stage1Active = false;
    }

    boolean stage2Ready() {
        return
            (leftStage2VelocitySignal.getValue() - linearVelocity.getValue()) < Constants.Launcher.stage2Tolerance 
            ;
    }



    void velocityParametersUpdated() {
        stage1Control.Velocity = linearVelocity.getValue() * Constants.Launcher.rotorToMeters;
        stage2Control.Velocity = linearVelocity.getValue() * Constants.Launcher.rotorToMeters;
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.teleoperated);
            add(RunContext.autonomous);
        }};
    }

    public void onStart(RunContext ctx) {
        stage1Control = new VelocityVoltage(0);
        stage2Control = new VelocityVoltage(0);

        stage1Control.UpdateFreqHz = 63;
        stage2Control.UpdateFreqHz = 63;

        linearVelocity = new Parameter<Double>(1.2);

        velocityParametersUpdated();

        linearVelocity.onValueUpdated = value -> velocityParametersUpdated();
        stopLauncher();
    }

    public void onLoop(RunContext ctx) {
        if(
            Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.launcherButton) 
            && Subsystems.intake.hasNote()
        ) {
            startLauncher();
        } else if (Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.launcherButton)) {
            stopLauncher();
        }

        if(stage2Ready() && !stage1Active) {
            stage1Active = true;
            Hardware.leftLauncherStage1.setControl(stage1Control);
        }
    }

    public void onStop() {}
}
