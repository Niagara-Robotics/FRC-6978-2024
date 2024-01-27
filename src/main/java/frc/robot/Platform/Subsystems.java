package frc.robot.Platform;

import frc.robot.Tasks.DifferentialDrive;
import frc.robot.Tasks.Intake;
import frc.robot.Tasks.Telemetry;
import frc.robot.Tasks.Tracking;
import frc.robot.Tasks.TwoSidedLauncher;

public class Subsystems {
    public static Telemetry telemetry = new Telemetry();

    public static Intake intake = new Intake();
    public static TwoSidedLauncher launcher = new TwoSidedLauncher();
    public static DifferentialDrive differentialDrive = new DifferentialDrive();
    public static Tracking tracking = new Tracking();
    
}
