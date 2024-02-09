package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Schedulers;
import frc.robot.Platform.Subsystems;
import frc.robot.Platform.Tasks;

public class Robot extends TimedRobot{
    Timer loopTimer;

    public Robot() {
        super(0.015);
    }
    
    @Override
    public void robotInit() {
        Hardware.configureHardware();
        Subsystems.telemetry.openSession("QSN", 1);
        
        Schedulers.idleScheduler.clear();
        for(IPeriodicTask task : Tasks.idleTasks) {
            Schedulers.idleScheduler.add(RunContext.disabled, task);
        }
        loopTimer = new Timer();
        loopTimer.reset();
        loopTimer.start();
    }

    @Override
    protected void loopFunc() {
        Subsystems.telemetry.openFrame();
        Subsystems.telemetry.pushDouble("robot.looptime", loopTimer.get());
        loopTimer.restart();
        super.loopFunc();
        Subsystems.telemetry.closeFrame();
        
    }

    @Override
    public void robotPeriodic() {
        Schedulers.idleScheduler.process(RunContext.disabled);
    }

    @Override
    public void teleopInit() {
        Schedulers.teleopScheduler.clear();
        for (IPeriodicTask task : Tasks.teleopTasks) {
            Schedulers.teleopScheduler.add(RunContext.teleoperated, task);
        }
    }

    @Override
    public void teleopPeriodic() {
        Schedulers.teleopScheduler.process(RunContext.teleoperated);
    }

    @Override
    public void autonomousInit() {
        Schedulers.autoScheduler.clear();
        for (IPeriodicTask task : Tasks.autonomousTasks) {
            Schedulers.autoScheduler.add(RunContext.autonomous, task);
        }
    }

    @Override
    public void autonomousPeriodic() {
        Schedulers.autoScheduler.process(RunContext.autonomous);
    }
    
    @Override
    public void disabledInit() {
        Schedulers.teleopScheduler.clear();
        Schedulers.autoScheduler.clear();
    }

    @Override
    public void endCompetition() {
        Subsystems.telemetry.closeSession();
        super.endCompetition();
    }
}