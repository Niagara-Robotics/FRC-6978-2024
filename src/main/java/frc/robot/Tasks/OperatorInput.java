package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Subsystems;

public class OperatorInput implements IPeriodicTask {
    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.disabled);
            add(RunContext.teleoperated);
            add(RunContext.autonomous);
        }};
    }

    public void onStart(RunContext ctx) {

    }

    public void onLoop(RunContext ctx) {
        if(Hardware.operatorStick.getRawButtonPressed(Constants.OperatorControls.configureSubShotButton)) {
            Subsystems.autoShot.setupSpeakerShot(1.350);
        }
        if(Hardware.operatorStick.getRawButtonPressed(Constants.OperatorControls.configurePodiumShotButton)) {
            Subsystems.autoShot.setupSpeakerShot(2.500);
        }
    }

    public void onStop() {
        
    }

    public void publishTelemetry() {
        /*if(Hardware.operatorStick.getRawButtonPressed(Constants.OperatorControls.configurePodiumShotButton)) {
            Subsystems.telemetry.pushBoolean("testval", true);
        }*/
    }
}
