package frc.robot.Tasks;

import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;

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
        if(Hardware.operatorStick.getRawButton(Constants.OperatorControls.climbUpButton) /*&& !beyondCatchPoint()*/) {
            //Hardware.liftMotor.set(ControlMode.Velocity, Constants.Lift.velocity);
            Hardware.liftMotor.set(ControlMode.PercentOutput, Constants.Lift.powerUp);
            Hardware.secondaryLiftMotor.set(ControlMode.PercentOutput, Constants.Lift.powerUp);
            Hardware.tertiaryLiftMotor.set(ControlMode.PercentOutput, Constants.Lift.powerUp);
            Hardware.quaternaryLiftMotor.set(ControlMode.PercentOutput, Constants.Lift.powerUp);
        } else if(Hardware.operatorStick.getRawButton(Constants.OperatorControls.climbReleaseButton) /*&& !beyondCatchPoint()*/) {
            //Hardware.liftMotor.set(ControlMode.PercentOutput, -Constants.Lift.power);
            Hardware.liftMotor.set(ControlMode.PercentOutput, -Constants.Lift.powerDown);
            Hardware.secondaryLiftMotor.set(ControlMode.PercentOutput, -Constants.Lift.powerDown);
            Hardware.tertiaryLiftMotor.set(ControlMode.PercentOutput, -Constants.Lift.powerDown);
            Hardware.quaternaryLiftMotor.set(ControlMode.PercentOutput, -Constants.Lift.powerDown);
        } else {
            Hardware.liftMotor.set(ControlMode.Disabled, 0);
            Hardware.secondaryLiftMotor.set(ControlMode.Disabled, 0);
            Hardware.tertiaryLiftMotor.set(ControlMode.Disabled, 0);
            Hardware.quaternaryLiftMotor.set(ControlMode.Disabled, 0);
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
