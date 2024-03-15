package frc.robot.Tasks;

import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import java.util.ArrayList;


public class Lift implements IPeriodicTask {
    
    public boolean beyondCatchPoint() {
        //return Hardware.liftMotor.getSelectedSensorPosition() > Constants.Lift.catchPoint;
        return Hardware.liftSensor.get();
    }

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
        if(Hardware.operatorStick.getRawButton(Constants.OperatorControls.climbUpButton) && !beyondCatchPoint()) {
            Hardware.liftMotor.set(ControlMode.Velocity, Constants.Lift.velocity);
            //Hardware.secondaryLiftMotor.set(ControlMode.PercentOutput, Constants.Lift.power);
        } else if(Hardware.operatorStick.getRawButton(Constants.OperatorControls.climbReleaseButton) && !beyondCatchPoint()) {
            Hardware.liftMotor.set(ControlMode.PercentOutput, -Constants.Lift.power);
            //Hardware.secondaryLiftMotor.set(ControlMode.PercentOutput, -Constants.Lift.power);
        } else {
            Hardware.liftMotor.set(ControlMode.Disabled, 0);
            //Hardware.secondaryLiftMotor.set(ControlMode.Disabled, 0);
        }
    }

    public void publishTelemetry() {
        // TODO Auto-generated method stub
        Subsystems.telemetry.pushDouble("lift_position", Hardware.liftMotor.getSelectedSensorPosition());
        Subsystems.telemetry.pushDouble("lift_velocity", Hardware.liftMotor.getSelectedSensorVelocity());
        Subsystems.telemetry.pushBoolean("lift_limitSwitch", Hardware.liftSensor.get());
    }

    public void onStop() {

    }
}
