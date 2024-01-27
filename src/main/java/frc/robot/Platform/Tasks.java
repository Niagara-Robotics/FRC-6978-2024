package frc.robot.Platform;

import frc.robot.Framework.IPeriodicTask;

public class Tasks {
    public static IPeriodicTask[] teleopTasks = {
        (IPeriodicTask)Subsystems.differentialDrive,
        (IPeriodicTask)Subsystems.intake,
    };

    public static IPeriodicTask[] autonomousTasks = {
        (IPeriodicTask)Subsystems.differentialDrive,

    };

    public static IPeriodicTask[] idleTasks = {
        (IPeriodicTask)Subsystems.tracking,
    };
}
