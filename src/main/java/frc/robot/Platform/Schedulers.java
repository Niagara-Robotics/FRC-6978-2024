package frc.robot.Platform;

import frc.robot.Framework.Scheduler;

public class Schedulers {
    public static Scheduler teleopScheduler = new Scheduler("teleop");
    public static Scheduler idleScheduler = new Scheduler("idle");
    public static Scheduler autoScheduler = new Scheduler("auto");
}
