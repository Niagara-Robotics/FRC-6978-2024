package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Schedulers;
import frc.robot.Platform.Tasks;

public class Robot extends TimedRobot{

    public Robot() {
        super(0.011);
    }
    
    @Override
    public void robotInit() {
        Hardware.configureHardware();
        Tasks.telemetry.openSession();
        
    }

    @Override
    protected void loopFunc() {
        Tasks.telemetry.openFrame();
        super.loopFunc();
        //Tasks.telemetry.closeFrame();
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
    public void disabledInit() {
        Schedulers.teleopScheduler.clear();
    }

    @Override
    public void endCompetition() {
        Tasks.telemetry.closeSession();
        super.endCompetition();
    }
}