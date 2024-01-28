package frc.robot.Platform;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Hardware {
    //Gyro
    public static AHRS navX = new AHRS();

    //Drive Motors
    public static TalonFX leftDriveLeader = new TalonFX(1, "rio");
    public static TalonFX leftDrive2 = new TalonFX(2, "rio");
    public static TalonFX rightDriveLeader = new TalonFX(3, "rio");
    public static TalonFX rightDrive2 = new TalonFX(4, "rio");

    public static TalonFX leftLauncherStage1 = new TalonFX(21);
    public static TalonFX leftLauncherStage2 = new TalonFX(20);

    public static TalonFX rightLauncherStage1 = new TalonFX(23);
    public static TalonFX rightLauncherStage2 = new TalonFX(22);

    public static TalonSRX intakeFloorRoller = new TalonSRX(10);
    public static TalonSRX intakeIndexerRoller = new TalonSRX(11);

    //Pneumatics
    public static Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    public static Solenoid driveGearShiftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    public static Solenoid gripperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    public static DigitalInput floorSensor = new DigitalInput(0);
    public static DigitalInput indexSensor = new DigitalInput(1);

    public static Counter noteExitCounter = new Counter(Counter.Mode.kTwoPulse);

    //Human input devices
    public static Joystick driverStick = new Joystick(0);
    public static Joystick operatorStick = new Joystick(1);

    public static DifferentialDriveKinematics kinematics;

    public static void configureHardware() {
        //Drive motors
        leftDriveLeader.setInverted(false);
        //leftDrive2.setInverted(false);
        rightDriveLeader.setInverted(true);
        //rightDrive2.setInverted(false);

        leftDrive2.setControl(new Follower(1, false));
        rightDrive2.setControl(new Follower(3, false));

        TalonFXConfiguration leftDriveConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration rightDriveConfiguration = new TalonFXConfiguration();

        leftDriveConfiguration.Slot0.kP = Constants.Drive.kP;
        leftDriveConfiguration.Slot0.kI = Constants.Drive.kI;
        leftDriveConfiguration.Slot0.kD = Constants.Drive.kD;
        leftDriveConfiguration.Slot0.kV = Constants.Drive.kVleft;
        leftDriveConfiguration.Slot0.kS = Constants.Drive.kSleft;
        leftDriveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rightDriveConfiguration.Slot0.kP = Constants.Drive.kP;
        rightDriveConfiguration.Slot0.kI = Constants.Drive.kI;
        rightDriveConfiguration.Slot0.kD = Constants.Drive.kD;
        rightDriveConfiguration.Slot0.kV = Constants.Drive.kVright;
        rightDriveConfiguration.Slot0.kS = Constants.Drive.kSright;
        rightDriveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightDriveConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftDriveLeader.getConfigurator().apply(leftDriveConfiguration);
        rightDriveLeader.getConfigurator().apply(rightDriveConfiguration);

        TalonFXConfiguration launcherConfiguration = new TalonFXConfiguration();
        launcherConfiguration.Slot0.kP = Constants.Launcher.kP;
        launcherConfiguration.Slot0.kV = Constants.Launcher.left_kV;
        launcherConfiguration.Slot0.kS = Constants.Launcher.kS;

        leftLauncherStage2.getConfigurator().apply(launcherConfiguration);

        launcherConfiguration.Slot0.kV = Constants.Launcher.right_kV;
        rightLauncherStage2.getConfigurator().apply(launcherConfiguration);

        rightLauncherStage2.setInverted(true);

        //other
        navX.reset();

        kinematics = new DifferentialDriveKinematics(Constants.Drive.trackWidth);

        intakeFloorRoller.setInverted(true);

        noteExitCounter.setUpSource(2);
        noteExitCounter.setUpSourceEdge(false, true);
    }
}
