package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Subsystems;

public class Watchdog implements IPeriodicTask {
    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.disabled);
            add(RunContext.teleoperated);
            add(RunContext.autonomous);
        }};
    }

    boolean redAlliance;

    //functions for checking if communication to substituent controllers is functioning

    public boolean driveHardwareAlive() {
        return(
            //Hardware.leftDriveLeader.isAlive() &&
            //Hardware.leftDrive2.isAlive() &&
            //Hardware.rightDriveLeader.isAlive() &&
            //Hardware.rightDrive2.isAlive()
            true //FIXME: drive hardware alive check
        );
    }

    public boolean liftHardwareAlive() {
        return(
            Hardware.liftMotor.getBusVoltage() > Constants.Watchdog.minBusVoltage &&
            Hardware.liftMotor.getFaults(new Faults()) == ErrorCode.OK &&
            Hardware.secondaryLiftMotor.getFaults(new Faults()) == ErrorCode.OK &&
            Hardware.tertiaryLiftMotor.getFaults(new Faults()) == ErrorCode.OK &&
            Hardware.quaternaryLiftMotor.getFaults(new Faults()) == ErrorCode.OK
        );
    }

    public boolean powerPathOkay() {
        return (
            !Hardware.PDH.getFaults().HardwareFault &&
            Hardware.PDH.getVoltage() > Constants.Watchdog.minBusVoltage
        );
    }

    public boolean configOkay() {
        return (
            driveHardwareAlive() &&
            Subsystems.tracking.poseGood() &&
            liftHardwareAlive() && 
            powerPathOkay()
        );
    }

    public boolean wrongZone() {
        return false;
    }

    public void onStart(RunContext ctx) {

    }

    public void onLoop(RunContext ctx) {

    }

    public void publishTelemetry() {
        SmartDashboard.putBoolean("watchdog_driveHardwareAlive", driveHardwareAlive());
        SmartDashboard.putBoolean("watchdog_liftHardwareAlive", liftHardwareAlive());
        SmartDashboard.putBoolean("watchdog_configOkay", configOkay());
    }

    public void onStop() {

    }
}
