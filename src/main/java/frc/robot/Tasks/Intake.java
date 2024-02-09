package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Subsystems;

public class Intake implements IPeriodicTask{

    public class IntakeState {
        public boolean floorSensor;
        //index sensor stops the intake, note is considered fully controlled
        public boolean indexSensor;

        public boolean floorRoller;
        public boolean indexRoller;

        //whether the intake is attempting to pick up a note
        public boolean active;

        public boolean hasNote;

        public IntakeState(boolean floor, boolean index) {
            floorSensor = floor;
            indexSensor = index;
            active = false;
            floorRoller = false;
            indexRoller = false;
        }
    }

    IntakeState currentState;
    IntakeState previousState;

    public Intake() {
        currentState = new IntakeState(false, false);
    }

    //Intake stops itself once a note is detected inside
    public boolean intakeNote() {
        //dont start the intake if the 
        if(currentState.indexSensor || hasNote()) return false;

        Hardware.intakeFloorRoller.set(ControlMode.PercentOutput, Constants.Intake.floorRollerPower);
        Hardware.intakeIndexerRoller.set(Constants.Intake.indexRollerPower);
    
        currentState.floorRoller = true;
        currentState.indexRoller = true;
        Subsystems.telemetry.pushEvent("intake.began");
        return true;
    }

    public boolean feedLauncher() {
        //if(currentState.indexSensor |! hasNote()) return false;
        if(currentState.active) return false;
        currentState.indexRoller = true;
        currentState.active = true;
        Hardware.intakeIndexerRoller.set(Constants.Intake.indexRollerFeedLauncherPower);
        return true;
    }

    public void cancelFeed() {
        if (currentState.indexRoller && currentState.active && !currentState.floorRoller) {
            idleIntake();
        }
    }

    public void idleIntake() {
        Hardware.intakeFloorRoller.set(ControlMode.Disabled, 0);
        Hardware.intakeIndexerRoller.set(0);
        currentState.floorRoller = false;
        currentState.indexRoller = false;
        currentState.active = false;
    }
    
    //returns true when the intake has finished its action and a note is inside the bot
    public boolean intakeFinished() {
        if(currentState.hasNote) return true;
        else if(currentState.indexSensor) {
            Subsystems.telemetry.pushEvent("intake.finished");
            idleIntake();
            currentState.hasNote = true;
            return true;
        }
        return false;
    }

    public boolean hasNote() {
        return currentState.hasNote;
    }

    //clears the hasNote flag(for use by the shooter)
    public void clearNote() {
        currentState.hasNote = false;
    }

    void evaluateSensors() {
        //TODO: remove previousState, unneccesary and unused
        if(currentState != null)
        previousState = currentState;

        currentState.floorSensor = Hardware.floorSensor.get();
        currentState.indexSensor = !Hardware.indexSensor.get();
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.disabled);
            add(RunContext.teleoperated);
            add(RunContext.autonomous);
        }};
    }

    public void onStart(RunContext ctx) {
        Hardware.intakeFloorRoller.set(ControlMode.Disabled, 0);
        Hardware.intakeIndexerRoller.set(0);
        evaluateSensors();
        currentState.hasNote = false;
        if (currentState.indexSensor) {
            currentState.hasNote = true;
            Subsystems.telemetry.pushEvent("intake.assumedNotePresent");
        }
    }

    public void onLoop(RunContext ctx) {
        evaluateSensors();
        Subsystems.telemetry.pushBoolean("intake.floorSensor", currentState.floorSensor);
        Subsystems.telemetry.pushBoolean("intake.indexSensor", currentState.indexSensor);
        Subsystems.telemetry.pushBoolean("intake.floorRoller", currentState.floorRoller);
        Subsystems.telemetry.pushBoolean("intake.indexRoller", currentState.indexRoller);
        Subsystems.telemetry.pushBoolean("intake.hasNote", currentState.hasNote);
        Subsystems.telemetry.pushBoolean("intake.finished", intakeFinished());
        Subsystems.telemetry.pushBoolean("intake.active", currentState.active);

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.intakeButton)) {
            intakeNote();
        } else if(Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.intakeButton)) {
            //idleIntake();
        }
    }

    public void onStop() {
        idleIntake();
    }
}
