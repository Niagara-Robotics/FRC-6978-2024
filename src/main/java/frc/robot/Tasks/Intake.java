package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

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

    VelocityVoltage indexRollerVelocityControl;
    VoltageOut indexRollerVoltageControl;
    NeutralOut indexRollerNeutralControl;

    public Intake() {
        currentState = new IntakeState(false, false);
        indexRollerVelocityControl = new VelocityVoltage(Constants.Intake.indexRollerVelocity);
        indexRollerVelocityControl.UpdateFreqHz = 200;

        indexRollerVoltageControl = new VoltageOut(Constants.Intake.indexRollerFeedLauncherPower);

        indexRollerNeutralControl = new NeutralOut();
    }

    //Intake stops itself once a note is detected inside
    public boolean intakeNote() {
        //dont start the intake if the 
        if(currentState.indexSensor || hasNote()) return false;

        Hardware.intakeFloorRoller.set(ControlMode.PercentOutput, Constants.Intake.floorRollerPower);
    
        Hardware.intakeIndexerRoller.setControl(indexRollerVelocityControl);

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
        Hardware.intakeIndexerRoller.setControl(indexRollerVoltageControl);
        return true;
    }

    public void cancelFeed() {
        if (currentState.indexRoller && currentState.active && !currentState.floorRoller) {
            idleIntake();
        }
    }

    public void idleIntake() {
        Hardware.intakeFloorRoller.set(ControlMode.Disabled, 0);
        Hardware.intakeIndexerRoller.setControl(indexRollerNeutralControl);

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

    boolean getIndexSensor() {
        return currentState.indexSensor;
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
        evaluateSensors();
        currentState.hasNote = false;
        if (currentState.indexSensor) {
            System.out.println("assuming note already loaded");
            currentState.hasNote = true;
            Subsystems.telemetry.pushEvent("intake.assumedNotePresent");
        }
        Hardware.intakeIndexerRoller.setControl(indexRollerNeutralControl);
    }

    public void onLoop(RunContext ctx) {
        evaluateSensors();


        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.intakeButton)) {
            intakeNote();
        } else if(Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.intakeButton)) {
            //idleIntake();
        }
        intakeFinished();
    }

    public void publishTelemetry() {
        Subsystems.telemetry.pushBoolean("intake.indexSensor", currentState.indexSensor);
        Subsystems.telemetry.pushBoolean("intake.floorRoller", currentState.floorRoller);
        Subsystems.telemetry.pushBoolean("intake.indexRoller", currentState.indexRoller);
        Subsystems.telemetry.pushBoolean("intake.hasNote", currentState.hasNote);
        Subsystems.telemetry.pushBoolean("intake.active", currentState.active);
        Subsystems.telemetry.pushDouble("intake.indexRollerVoltage", Hardware.intakeIndexerRoller.getMotorVoltage().getValue());
        Subsystems.telemetry.pushDouble("intake.indexRollerVelocity", Hardware.intakeIndexerRoller.getVelocity().getValue());
        Subsystems.telemetry.pushDouble("intake.floorRollerVoltage", Hardware.intakeFloorRoller.getMotorOutputVoltage());
    }

    public void onStop() {
        idleIntake();
    }
}
