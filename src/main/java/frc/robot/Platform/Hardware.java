package frc.robot.Platform;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI.Port;

public class Hardware {
    //Gyro
    public static AHRS navX = new AHRS(Port.kMXP, (byte)200);

    //Drive Motors
    public static TalonFX leftDriveLeader = new TalonFX(1, "rio");
    public static TalonFX leftDrive2 = new TalonFX(2, "rio");
    public static TalonFX rightDriveLeader = new TalonFX(3, "rio");
    public static TalonFX rightDrive2 = new TalonFX(4, "rio");

    public static TalonFX leftLauncherStage1 = new TalonFX(21);
    public static TalonFX leftLauncherStage2 = new TalonFX(20);

    public static TalonFX rightLauncherStage1 = new TalonFX(23);
    public static TalonFX rightLauncherStage2 = new TalonFX(22);

    public static TalonFX launcherTiltMotor = new TalonFX(30);

    public static TalonSRX intakeFloorRoller = new TalonSRX(10);
    public static TalonSRX liftMotor = new TalonSRX(11);
    public static TalonSRX secondaryLiftMotor = new TalonSRX(12);
    public static TalonSRX tertiaryLiftMotor = new TalonSRX(13);
    public static TalonSRX quaternaryLiftMotor = new TalonSRX(14);

    public static TalonFX intakeIndexerRoller = new TalonFX(40);

    //Pneumatics

    public static DigitalInput liftSensor = new DigitalInput(0);
    public static DigitalInput indexSensor = new DigitalInput(1);

    public static Counter noteExitCounter = new Counter(Counter.Mode.kTwoPulse);

    //Human input devices
    public static Joystick driverStick = new Joystick(0);
    public static Joystick operatorStick = new Joystick(1);
    //public static Joystick guestStick = new Joystick(2);

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
        leftDriveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftDriveConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.Drive.supplyCurrentLimit;

        rightDriveConfiguration.Slot0.kP = Constants.Drive.kP;
        rightDriveConfiguration.Slot0.kI = Constants.Drive.kI;
        rightDriveConfiguration.Slot0.kD = Constants.Drive.kD;
        rightDriveConfiguration.Slot0.kV = Constants.Drive.kVright;
        rightDriveConfiguration.Slot0.kS = Constants.Drive.kSright;
        rightDriveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightDriveConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightDriveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightDriveConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.Drive.supplyCurrentLimit;
        //rightDriveConfiguration.CurrentLimits.SupplyCurrentThreshold = Constants.Drive.supplyCurrentLimit;

        leftDriveLeader.getConfigurator().apply(leftDriveConfiguration);
        rightDriveLeader.getConfigurator().apply(rightDriveConfiguration);

        leftDrive2.getConfigurator().apply(leftDriveConfiguration);
        rightDrive2.getConfigurator().apply(rightDriveConfiguration);

        leftDriveLeader.setPosition(0);
        rightDriveLeader.setPosition(0);

        TalonFXConfiguration launcherConfiguration = new TalonFXConfiguration();
        launcherConfiguration.Slot0.kP = Constants.Launcher.kP;
        launcherConfiguration.Slot0.kV = Constants.Launcher.left_kV;
        launcherConfiguration.Slot0.kS = Constants.Launcher.kS;
        launcherConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.Launcher.maxWheelCurrent;
        launcherConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        //75.8, 
        leftLauncherStage2.getConfigurator().apply(launcherConfiguration);
        leftLauncherStage1.getConfigurator().apply(launcherConfiguration);
        rightLauncherStage1.getConfigurator().apply(launcherConfiguration);

        launcherConfiguration.Slot0.kV = Constants.Launcher.right_kV;
        rightLauncherStage2.getConfigurator().apply(launcherConfiguration);

        rightLauncherStage2.setInverted(true);
        rightLauncherStage1.setInverted(true);

        TalonFXConfiguration launcherTiltConfiguration = new TalonFXConfiguration();
        launcherTiltConfiguration.Feedback.SensorToMechanismRatio = Constants.Launcher.tiltRotorToMechanismRatio;
        launcherTiltConfiguration.Voltage.PeakForwardVoltage = Constants.Launcher.tiltMaxOutputUp;
        launcherTiltConfiguration.Voltage.PeakReverseVoltage = -Constants.Launcher.tiltMaxOutputDown;
        launcherTiltConfiguration.Slot0.kP = Constants.Launcher.tiltKp;
        launcherTiltConfiguration.Slot0.kD = Constants.Launcher.tiltKd;
        launcherTiltConfiguration.Slot0.kG = Constants.Launcher.tiltKg;
        launcherTiltConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        launcherTiltConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        launcherTiltMotor.getConfigurator().apply(launcherTiltConfiguration);
        launcherTiltMotor.setPosition(Constants.Launcher.tiltDefaultPosition);

        //other
        navX.reset();

        kinematics = new DifferentialDriveKinematics(Constants.Drive.trackWidth);

        TalonFXConfiguration intakeIndexerRollerConfig = new TalonFXConfiguration();
        intakeIndexerRollerConfig.Slot0.kP = Constants.Intake.indexer_kP;
        intakeIndexerRollerConfig.Slot0.kS = Constants.Intake.indexer_kS;
        intakeIndexerRollerConfig.Slot0.kV = Constants.Intake.indexer_kV;
        intakeIndexerRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeIndexerRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        intakeIndexerRoller.getConfigurator().apply(intakeIndexerRollerConfig);

        intakeFloorRoller.setInverted(true);

        noteExitCounter.setUpSource(2);
        noteExitCounter.setUpSourceEdge(false, true);

        liftMotor.setSelectedSensorPosition(0);
        liftMotor.config_kF(0, Constants.Lift.kF);
        liftMotor.config_kP(0, Constants.Lift.kP);

        //liftMotor.configContinuousCurrentLimit(25)

        secondaryLiftMotor.set(ControlMode.Follower, 11);
        quaternaryLiftMotor.setInverted(true);
    }
}
