package frc.robot.Platform;

import frc.robot.Framework.IPeriodicTask;

public class Tasks {
    public static IPeriodicTask[] telemetryTasks = {
        (IPeriodicTask)Subsystems.differentialDrive,
        (IPeriodicTask)Subsystems.intake,
        (IPeriodicTask)Subsystems.launcher,
        (IPeriodicTask)Subsystems.driverInput,
        (IPeriodicTask)Subsystems.tracking,
        (IPeriodicTask)Subsystems.lift,
        (IPeriodicTask)Subsystems.autoShot,
    };
    
    public static IPeriodicTask[] teleopTasks = {
        (IPeriodicTask)Subsystems.differentialDrive,
        (IPeriodicTask)Subsystems.intake,
        (IPeriodicTask)Subsystems.launcher,
        (IPeriodicTask)Subsystems.driverInput,
        (IPeriodicTask)Subsystems.autoShot,
        (IPeriodicTask)Subsystems.lift,
        (IPeriodicTask)Subsystems.operatorInput,
    };

    public static IPeriodicTask[] autonomousTasks = {
        (IPeriodicTask)Subsystems.differentialDrive,
        (IPeriodicTask)Subsystems.intake,
        (IPeriodicTask)Subsystems.auto,
        (IPeriodicTask)Subsystems.launcher,
        (IPeriodicTask)Subsystems.autoShot,
    };

    public static IPeriodicTask[] idleTasks = {
        (IPeriodicTask)Subsystems.tracking,
    };
}
