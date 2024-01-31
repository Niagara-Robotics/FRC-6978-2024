package frc.robot.Platform;

import frc.robot.Framework.IPeriodicTask;

public class Tasks {
    public static IPeriodicTask[] teleopTasks = {
        (IPeriodicTask)Subsystems.differentialDrive,
        (IPeriodicTask)Subsystems.intake,
        (IPeriodicTask)Subsystems.launcher,
    };

    public static IPeriodicTask[] autonomousTasks = {
        (IPeriodicTask)Subsystems.differentialDrive,
        (IPeriodicTask)Subsystems.intake,
        (IPeriodicTask)Subsystems.auto,
        (IPeriodicTask)Subsystems.launcher,
    };

    public static IPeriodicTask[] idleTasks = {
        (IPeriodicTask)Subsystems.tracking,
    };
}
