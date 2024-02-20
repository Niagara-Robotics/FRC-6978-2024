package frc.robot.Tasks;

import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import java.util.ArrayList;


public class Lift implements IPeriodicTask {
    


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
        if(Hardware.operatorStick.getRawButton(Constants.OperatorControls.climbUpButton)) {
            Hardware.liftMotor.set(ControlMode.PercentOutput, Constants.Lift.power);
            Hardware.secondaryLiftMotor.set(ControlMode.PercentOutput, Constants.Lift.power);
        } else if(Hardware.operatorStick.getRawButton(Constants.OperatorControls.climbReleaseButton)) {
            Hardware.liftMotor.set(ControlMode.PercentOutput, -Constants.Lift.power);
            Hardware.secondaryLiftMotor.set(ControlMode.PercentOutput, -Constants.Lift.power);
        } else {
            Hardware.liftMotor.set(ControlMode.Disabled, 0);
            Hardware.secondaryLiftMotor.set(ControlMode.Disabled, 0);
        }
    }

    public void publishTelemetry() {
        // TODO Auto-generated method stub
        
    }

    public void onStop() {

    }
}
