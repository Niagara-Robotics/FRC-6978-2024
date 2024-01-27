package frc.robot.Platform;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Compressor;
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

    //Aft lift mechanism
    public static TalonFX armLiftMotor = new TalonFX(5);
    //Fwd cable lift
    public static TalonFX armCableMotor = new TalonFX(50);


    //Pneumatics
    public static Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    public static Solenoid driveGearShiftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    public static Solenoid gripperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

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

        //other
        navX.reset();

        kinematics = new DifferentialDriveKinematics(Constants.Drive.trackWidth);
    }
}
