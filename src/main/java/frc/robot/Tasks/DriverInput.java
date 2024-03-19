package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;
import java.util.random.RandomGenerator;

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
        driveHandle = Subsystems.differentialDrive.wantedChassisSpeeds.getHandle("driverInput");
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

    public void useMainDriverStick() {
        double x = Hardware.driverStick.getRawAxis(Constants.DriverControls.steeringAxis);
        double y = (Hardware.driverStick.getRawAxis(Constants.DriverControls.forwardAxis) + 1 )/2 - 
            (Hardware.driverStick.getRawAxis(Constants.DriverControls.reverseAxis) + 1 )/2;
        

        if(autoAlign) {
 
        } else if(autoNote) {
            Subsystems.telemetry.pushDouble("autoNote.delta", Subsystems.tracking.noteTargetX);
            Subsystems.telemetry.pushDouble("autoNote.output", noteController.process(Subsystems.tracking.noteTargetX));

            double output = noteController.process(Subsystems.tracking.noteTargetX);
            if(Subsystems.tracking.noteTargetY > 1) {output = 0.0;};

            driveHandle.set(new ChassisSpeeds(0.8,0, output));

        } else driveStickVelocity(x, y, 3, 3.65);

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
        
        noteController = new PIDController(0.020, 0.0, 0.5);
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
            driveHandle.release();
            autoAlign = true;
            Subsystems.autoShot.fullAutoLaunch();
        } else if(Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.autoAlignButton)) {
            autoAlign = false;
            driveHandle.takeControl(false);
            driveHandle.set(new ChassisSpeeds(0,0,0));
            Subsystems.autoShot.cancelAutoLaunch();
        }

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.autoNoteButton)) {
            noteController.init();
            noteController.set(0.0);
            noteController.setLimit(1.1);
            autoNote = true;
            Subsystems.intake.intakeNote();
        } else if(Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.autoNoteButton) || 
            (Subsystems.intake.intakeFinished() &&autoNote)) {
            autoNote = false;
            driveHandle.set(new ChassisSpeeds(0,0,0));
        }
        
        if(guestActive) {
            useGuestStick();
        } else {
            useMainDriverStick();
        }

        //tiltHandle.takeControl(false);

        if(Hardware.driverStick.getRawButtonPressed(1)) {
            tiltHandle.takeControl(false);
            tiltHandle.set(-0.1);

            if(RandomGenerator.getDefault().nextDouble() > 0.98) {
                System.out.println("Parked tile mechanism");
                System.out.println("*tilte");
                System.out.println("*tilt");
                System.out.println("damn autocorrect");
            } else {
                System.out.println("Parked tilt mechanism");
            }
        } else if (Hardware.driverStick.getRawButtonPressed(4)) {
            tiltHandle.set(0.023);
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
