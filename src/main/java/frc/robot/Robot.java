package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Schedulers;
import frc.robot.Platform.Subsystems;
import frc.robot.Platform.Tasks;

public class Robot extends TimedRobot{
    Timer loopTimer;

    public Robot() {
        super(0.005);
    }
    
    @Override
    public void robotInit() {
        Hardware.configureHardware();
        Subsystems.telemetry.openSession("QSN", 1);
        
        //Schedulers.idleScheduler.clear();
        for(IPeriodicTask task : Tasks.idleTasks) {
            Schedulers.idleScheduler.add(task);
        }

        for(IPeriodicTask task: Tasks.telemetryTasks) {
            Schedulers.teleScheduler.add(task);
        }

        loopTimer = new Timer();
        loopTimer.reset();
        loopTimer.start();
    }

    @Override
    protected void loopFunc() {
        Subsystems.telemetry.openFrame();
        loopTimer.restart();
        super.loopFunc();
        Subsystems.telemetry.pushDouble("robot.looptime", loopTimer.get());
        Subsystems.telemetry.closeFrame();
        
    }

    @Override
    public void robotPeriodic() {
        Schedulers.teleScheduler.run();
        Schedulers.idleScheduler.run();
    }

    @Override
    public void teleopInit() {
        //Schedulers.teleopScheduler.clear();
        for (IPeriodicTask task : Tasks.teleopTasks) {
            Schedulers.teleopScheduler.add(task);
        }
    }

    @Override
    public void teleopPeriodic() {
        Schedulers.teleopScheduler.run();
    }

    @Override
    public void autonomousInit() {
        //Schedulers.autoScheduler.clear();
        for (IPeriodicTask task : Tasks.autonomousTasks) {
            Schedulers.autoScheduler.add(task);
        }
    }

    @Override
    public void autonomousPeriodic() {
        Schedulers.autoScheduler.run();
    }
    
    @Override
    public void disabledInit() {
        Schedulers.teleopScheduler.clear();
        Schedulers.autoScheduler.clear();

        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
            if(alliance.get() == Alliance.Red) {
                Subsystems.illumination.setBreathing((byte)0, (short)8000, 130, 0, 0);
                Subsystems.illumination.setBreathing((byte)1, (short)8000, 130, 0, 0);
            } else {
                Subsystems.illumination.setBreathing((byte)0, (short)8000, 0, 0, 130);
                Subsystems.illumination.setBreathing((byte)1, (short)8000, 0, 0, 130);
            }
        }
    }

    @Override
    public void endCompetition() {
        Subsystems.telemetry.closeSession();
        super.endCompetition();
    }
}