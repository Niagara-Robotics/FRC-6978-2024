package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.PIDController;
import frc.robot.Framework.ParameterHandle;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Subsystems;

public class DriverInput implements IPeriodicTask {
    boolean guestActive = false;
    boolean autoAlign = false;

    PIDController alignmentController;

    ParameterHandle<Double> tiltHandle;
    
    public DriverInput() {
        tiltHandle = Subsystems.launcher.tilt.getHandle("driver");
    }

    void driveStickVelocity(double x, double y, double xMultiplier, double yMultiplier) {
        x = (Math.abs(x) > Constants.Drive.deadZone)? 
            ((x > 0)? 
                ((x-Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) :
                ((x+Constants.Drive.deadZone)/(1-Constants.Drive.deadZone))
            ) 
            : 0;

        y = (Math.abs(y) > Constants.Drive.deadZone)? 
            ((y > 0)? 
                ((y-Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) :
                ((y+Constants.Drive.deadZone)/(1-Constants.Drive.deadZone))
            ) 
            : 0;

        x *= xMultiplier;
        y *= yMultiplier;

        Subsystems.telemetry.pushDouble("DriveX", x);
        Subsystems.telemetry.pushDouble("DriveY", y);

        ChassisSpeeds speeds = new ChassisSpeeds(y, 0, -x);

        Subsystems.differentialDrive.driveChassisSpeeds(speeds);
    }

    public void useMainDriverStick() {
        double x = Hardware.driverStick.getRawAxis(Constants.DriverControls.steeringAxis);
        double y = (Hardware.driverStick.getRawAxis(Constants.DriverControls.forwardAxis) + 1 )/2 - 
            (Hardware.driverStick.getRawAxis(Constants.DriverControls.reverseAxis) + 1 )/2;
        

        if(autoAlign) {
                Subsystems.telemetry.pushDouble("autoAlign.delta", Subsystems.tracking.shotTargetX - 300);
                Subsystems.telemetry.pushDouble("autoAlign.output", alignmentController.process(Subsystems.tracking.shotTargetX - 300));

                Subsystems.differentialDrive.driveChassisSpeeds(new ChassisSpeeds(0,0, alignmentController.process(Subsystems.tracking.shotTargetX - 300)));
        } else driveStickVelocity(x, y, 2, 2.2);

        //shoot button
        if(Hardware.driverStick.getRawButtonPressed(Constants.OperatorControls.launcherButton)) {
            Subsystems.launcher.launchNote();
        } else if (Hardware.driverStick.getRawButtonReleased(Constants.OperatorControls.launcherButton)) {
            Subsystems.launcher.stopLauncher();
        }
    }

    public void useGuestStick() {
        double x = Hardware.guestStick.getRawAxis(Constants.DriverControls.steeringAxis);
        double y = (Hardware.guestStick.getRawAxis(Constants.DriverControls.forwardAxis) + 1 )/2 - 
            (Hardware.guestStick.getRawAxis(Constants.DriverControls.reverseAxis) + 1 )/2;

        driveStickVelocity(x, y, 1., 0.7);

        if(Hardware.guestStick.getRawButtonPressed(Constants.OperatorControls.launcherButton)) {
            Subsystems.launcher.launchNote();
        } else if (Hardware.guestStick.getRawButtonReleased(Constants.OperatorControls.launcherButton)) {
            Subsystems.launcher.stopLauncher();
        }
    }
    
    
    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.disabled);
            add(RunContext.teleoperated);
            add(RunContext.autonomous);
        }};
    }

    public void onStart(RunContext ctx) {
        guestActive = false;
        alignmentController = new PIDController(0.0075, 0.0, 0.5);
        if(!tiltHandle.hasControl()) tiltHandle.takeControl(false);
    }

    public void onLoop(RunContext ctx) {
        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.transferControlButton)) {
            if(guestActive) {
                Subsystems.launcher.stopLauncher();
                Subsystems.intake.idleIntake();
                guestActive = false;
            } else {
                Subsystems.launcher.stopLauncher();
                Subsystems.intake.idleIntake();
                guestActive = true;
            }
            Subsystems.telemetry.pushBoolean("driverHasControl", !guestActive);
        }

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.autoAlignButton)) {
            alignmentController.init();
            alignmentController.set(0);
            alignmentController.setLimit(1.1);
            autoAlign = true;
            Subsystems.autoShot.fullAutoLaunch();
        } else if(Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.autoAlignButton)) {
            autoAlign = false;
            Subsystems.differentialDrive.driveChassisSpeeds(new ChassisSpeeds(0,0,0));
        }
        
        if(guestActive) {
            useGuestStick();
        } else {
            useMainDriverStick();
        }

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.tiltTakeover)) 
        tiltHandle.takeControl(false);

        if(Hardware.driverStick.getRawButtonPressed(1)) {
            tiltHandle.takeControl(false);
            tiltHandle.set(0.0);
        } else if (Hardware.driverStick.getRawButtonPressed(4)) {
            tiltHandle.set(0.023);
        }

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.ampShotButton)) {
            Subsystems.autoShot.setupAmpShot();
        }
        
        //new SPI(SPI.Port.kOnboardCS0).;
    }

    public void publishTelemetry() {
        Subsystems.telemetry.pushBoolean("driverInput.hasTiltControl", tiltHandle.hasControl());
    }

    public void onStop() {

    }
}
