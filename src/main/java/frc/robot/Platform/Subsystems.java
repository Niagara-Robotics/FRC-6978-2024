package frc.robot.Platform;

import frc.robot.Tasks.DifferentialDrive;
import frc.robot.Tasks.Intake;
import frc.robot.Tasks.Telemetry;
import frc.robot.Tasks.Tracking;

public class Subsystems {
    public static Intake intake = new Intake();
    public static DifferentialDrive differentialDrive = new DifferentialDrive();
    public static Tracking tracking = new Tracking();
    public static Telemetry telemetry = new Telemetry();
}
