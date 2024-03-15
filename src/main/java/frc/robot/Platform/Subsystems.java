package frc.robot.Platform;

import frc.robot.Tasks.Lift;
import frc.robot.Tasks.OperatorInput;
import frc.robot.Tasks.AutoPilot;
import frc.robot.Tasks.Auto;
import frc.robot.Tasks.AutoShot;
import frc.robot.Tasks.DifferentialDrive;
import frc.robot.Tasks.DriverInput;
import frc.robot.Tasks.Illumination;
import frc.robot.Tasks.Intake;
import frc.robot.Tasks.Telemetry;
import frc.robot.Tasks.Tracking;
import frc.robot.Tasks.TwoSidedLauncher;

public class Subsystems {
    public static Telemetry telemetry = new Telemetry();

    public static Illumination illumination = new Illumination();

    public static Intake intake = new Intake();
    public static OperatorInput operatorInput = new OperatorInput();
    public static TwoSidedLauncher launcher = new TwoSidedLauncher();
    public static DifferentialDrive differentialDrive = new DifferentialDrive();
    public static Lift lift = new Lift();
    public static Tracking tracking = new Tracking();
    
    public static DriverInput driverInput = new DriverInput();

    public static AutoShot autoShot = new AutoShot();

    public static AutoPilot autoPilot = new AutoPilot();

    public static Auto auto = new Auto();



}
