package frc.robot.Platform;

import frc.robot.Framework.RunContext;
import frc.robot.Framework.Scheduler;

public class Schedulers {
    public static Scheduler teleopScheduler = new Scheduler("teleop", false, RunContext.teleoperated);
    public static Scheduler idleScheduler = new Scheduler("idle", false, RunContext.disabled);
    public static Scheduler autoScheduler = new Scheduler("auto", false, RunContext.autonomous);

    public static Scheduler teleScheduler = new Scheduler("telemetry", true, RunContext.disabled);
}
