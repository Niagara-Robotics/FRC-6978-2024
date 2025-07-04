package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    boolean autoNote = false;

    PIDController noteController;

    ParameterHandle<Double> tiltHandle;

    ParameterHandle<ChassisSpeeds> driveHandle;

    long lastSendTS;

    public DriverInput() {
        tiltHandle = Subsystems.launcher.tilt.getHandle("driver");
        driveHandle = Subsystems.drive.wantedChassisSpeeds.getHandle("driverInput");
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

        Subsystems.telemetry.pushDouble("driverInput_driveX", x);
        Subsystems.telemetry.pushDouble("driverInput_driveY", y);

        ChassisSpeeds speeds = new ChassisSpeeds(y, 0, -x);

        driveHandle.set(speeds);
    }

    void driveStickVelocitySwerve(double x, double y, double w, double xyMultiplier, double wMultiplier) {
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

        w = (Math.abs(w) > Constants.Drive.deadZone)? 
            ((w > 0)? 
                ((w-Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) :
                ((w+Constants.Drive.deadZone)/(1-Constants.Drive.deadZone))
            ) 
            : 0;

        x *= xyMultiplier;
        y *= xyMultiplier;
        w *= wMultiplier;

        Subsystems.telemetry.pushDouble("driverInput_driveX", x);
        Subsystems.telemetry.pushDouble("driverInput_driveY", y);
        Subsystems.telemetry.pushDouble("driverInput_driveW", w);

        ChassisSpeeds speeds = new ChassisSpeeds(y, x, w);

        driveHandle.set(speeds);
    }

    public void useMainDriverStick() {
        double x = -Hardware.driverStick.getRawAxis(Constants.DriverControls.strafeAxis);
        double y = -Hardware.driverStick.getRawAxis(Constants.DriverControls.forwardAxis);
        double w = Hardware.driverStick.getRawAxis(Constants.DriverControls.rotationAxis);
        

        if(autoAlign) {
 
        } else if(autoNote) {
            Subsystems.telemetry.pushDouble("autoNote.delta", Subsystems.tracking.noteTargetX);
            Subsystems.telemetry.pushDouble("autoNote.output", noteController.process(Subsystems.tracking.noteTargetX));

            double output = noteController.process(Subsystems.tracking.noteTargetX);
            if(Subsystems.tracking.noteTargetY > 1) {output = 0.0;};
            //FIXME:change back to 0.8
            driveHandle.set(new ChassisSpeeds(0.0,0, output));

        } else driveStickVelocitySwerve(x, y, w, 0.25, 0.5);

        //shoot button
        if(Hardware.driverStick.getRawButtonPressed(Constants.OperatorControls.launcherButton)) {
            Subsystems.launcher.launchNote();
        } else if (Hardware.driverStick.getRawButtonReleased(Constants.OperatorControls.launcherButton)) {
            Subsystems.launcher.stopLauncher();
        }
    }

    /*public void useGuestStick() {
        double x = Hardware.guestStick.getRawAxis(Constants.DriverControls.steeringAxis);
        double y = (Hardware.guestStick.getRawAxis(Constants.DriverControls.forwardAxis) + 1 )/2 - 
            (Hardware.guestStick.getRawAxis(Constants.DriverControls.reverseAxis) + 1 )/2;

        driveStickVelocity(x, y, 1., 0.7);

        if(Hardware.guestStick.getRawButtonPressed(Constants.OperatorControls.launcherButton)) {
            Subsystems.launcher.launchNote();
        } else if (Hardware.guestStick.getRawButtonReleased(Constants.OperatorControls.launcherButton)) {
            Subsystems.launcher.stopLauncher();
        }
    }*/
    
    
    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.disabled);
            add(RunContext.teleoperated);
            add(RunContext.autonomous);
        }};
    }

    public void onStart(RunContext ctx) {
        guestActive = false;
        
        noteController = new PIDController(0.020, 0.0, 0.5);
        if(!tiltHandle.hasControl()) tiltHandle.takeControl(false);
        driveHandle.takeControl(false);
    }

    public void onLoop(RunContext ctx) {
        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.transferControlButton)) {
            Subsystems.autoShot.spitNote();
        }

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.autoAlignButton)) {
            driveHandle.release();
            autoAlign = true;
            Subsystems.autoShot.fullAutoLaunch();
        } else if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.autoPassButton)) {
            driveHandle.release();
            autoAlign = true;
            Subsystems.autoShot.autoPass();
        } else if(Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.autoAlignButton) ||
            Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.autoPassButton)) {
            autoAlign = false;
            driveHandle.takeControl(false);
            driveHandle.set(new ChassisSpeeds(0,0,0));
            Subsystems.autoShot.cancelAutoLaunch();
        }

        /*if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.autoNoteButton)) {
            noteController.init();
            noteController.set(0.0);
            noteController.setLimit(1.1);
            autoNote = true;
            Subsystems.intake.intakeNote();
        } else if(Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.autoNoteButton) || 
            (Subsystems.intake.intakeFinished() &&autoNote)) {
            autoNote = false;
            driveHandle.set(new ChassisSpeeds(0,0,0));
        }*/
        
        if(guestActive) {
            //useGuestStick();
        } else {
            useMainDriverStick();
        }

        //tiltHandle.takeControl(false);

        /*if(Hardware.driverStick.getRawButtonPressed(1)) {
            tiltHandle.takeControl(false);
            tiltHandle.set(-0.1);
        } else if (Hardware.driverStick.getRawButtonPressed(4)) {
            tiltHandle.takeControl(false);
            tiltHandle.set(0.5);
        }*/

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.passButton)) {
            Subsystems.autoShot.passNote();
        } else if(Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.passButton)) {
            Subsystems.autoShot.cancelAutoLaunch();
        }

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.ampShotButton)) {
            Subsystems.autoShot.setupAmpShot();
        }

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.trapShotButton)) {
            Subsystems.autoShot.setupTrapShot();
        }

        if(Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.trapShotButton) ||
            Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.ampShotButton)) {
            Subsystems.autoShot.cancelAutoLaunch();
            driveHandle.takeControl(false);
            driveHandle.set(new ChassisSpeeds(0,0,0));
        }

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.intakeButton)) {
            Subsystems.intake.intakeNote();
        } else if(Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.intakeButton)) {
            //idleIntake();
        }
        
        
    }

    public void publishTelemetry() {
        Subsystems.telemetry.pushBoolean("driverInput_hasTiltControl", tiltHandle.hasControl());

    }

    public void onStop() {

    }
}
