package frc.robot.Tasks;

import java.security.PrivilegedActionException;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.Parameter;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;


public class SwerveDrive implements IPeriodicTask {

    private final SwerveModuleConstantsFactory constantCreator = new SwerveModuleConstantsFactory()
        .withDriveMotorGearRatio(Constants.SwerveDrive.driveGearRatio)
        .withSteerMotorGearRatio(Constants.SwerveDrive.steerGearRatio)
        .withWheelRadius(Constants.SwerveDrive.wheelRadius)
        .withSlipCurrent(Constants.SwerveDrive.slipCurrent)
        .withSteerMotorGains(Constants.SwerveDrive.steerGains)
        .withDriveMotorGains(Constants.SwerveDrive.driveGains)
        .withSteerMotorClosedLoopOutput(Constants.SwerveDrive.steerClosedLoopOutputType)
        .withDriveMotorClosedLoopOutput(Constants.SwerveDrive.driveClosedLoopOutputType)
        .withSpeedAt12VoltsMps(Constants.SwerveDrive.freeSpeedAt12Volts)
        .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
        .withCouplingGearRatio(Constants.SwerveDrive.coupleRatio)
        .withSteerMotorInverted(Constants.SwerveDrive.steeringInvert)
        .withDriveMotorInitialConfigs(Constants.SwerveDrive.driveInitialConfigs)
        .withSteerMotorInitialConfigs(Constants.SwerveDrive.steerInitialConfigs)
        .withCANcoderInitialConfigs(Constants.SwerveDrive.cancoderInitialConfigs);

    private final SwerveModuleConstants constants[] = {constantCreator.createModuleConstants(
        Constants.SwerveDrive.frontLeftSteerId, 
        Constants.SwerveDrive.frontLeftDriveId, 
        Constants.SwerveDrive.frontLeftEncoderId, 
        Constants.SwerveDrive.frontLeftencoderOffset, 
        Constants.SwerveDrive.frontLeftPositionX, 
        Constants.SwerveDrive.frontLeftPositionY, 
        Constants.SwerveDrive.invertLeftSide), 
        
        constantCreator.createModuleConstants(
        Constants.SwerveDrive.frontRightSteerId, 
        Constants.SwerveDrive.frontRightDriveId, 
        Constants.SwerveDrive.frontRightEncoderId, 
        Constants.SwerveDrive.frontRightencoderOffset, 
        Constants.SwerveDrive.frontRightPositionX, 
        Constants.SwerveDrive.frontRightPositionY, 
        Constants.SwerveDrive.invertRightSide),
    
        constantCreator.createModuleConstants(
        Constants.SwerveDrive.backLeftSteerId, 
        Constants.SwerveDrive.backLeftDriveId, 
        Constants.SwerveDrive.backLeftEncoderId, 
        Constants.SwerveDrive.backLeftencoderOffset, 
        Constants.SwerveDrive.backLeftPositionX, 
        Constants.SwerveDrive.backLeftPositionY, 
        Constants.SwerveDrive.invertLeftSide),
    
        constantCreator.createModuleConstants(
        Constants.SwerveDrive.backRightSteerId, 
        Constants.SwerveDrive.backRightDriveId, 
        Constants.SwerveDrive.backRightEncoderId, 
        Constants.SwerveDrive.backRightencoderOffset, 
        Constants.SwerveDrive.backRightPositionX, 
        Constants.SwerveDrive.backRightPositionY, 
        Constants.SwerveDrive.invertRightSide)
    };

    private ArrayList<SwerveModule> modules;

    SwerveDriveKinematics kinematics;

    List<StructPublisher<SwerveModuleState>> moduleStatePublishers;
    List<StructPublisher<SwerveModuleState>> moduleRequestPublishers;

    Parameter<ChassisSpeeds> wantedChassisSpeeds;

    public SwerveDrive() {
        modules = new ArrayList<SwerveModule>();
        for (SwerveModuleConstants swerveModuleConstants : constants) {
            modules.add(new SwerveModule(swerveModuleConstants, Constants.SwerveDrive.CANBusName));
        }

        Translation2d m_frontLeftLocation = new Translation2d(constants[0].LocationX, constants[0].LocationY);
        Translation2d m_frontRightLocation = new Translation2d(constants[0].LocationX, constants[0].LocationY);
        Translation2d m_backLeftLocation = new Translation2d(constants[0].LocationX, constants[0].LocationY);
        Translation2d m_backRightLocation = new Translation2d(constants[0].LocationX, constants[0].LocationY);

        kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    
        moduleStatePublishers = new ArrayList<StructPublisher<SwerveModuleState>>();
        for(int i=0; i<modules.size(); i++) {
            moduleStatePublishers.add(NetworkTableInstance.getDefault().getTable("Drive").getStructTopic("moduleState_" + i, SwerveModuleState.struct).publish());
        }

        moduleRequestPublishers = new ArrayList<StructPublisher<SwerveModuleState>>();
        for(int i=0; i<modules.size(); i++) {
            moduleRequestPublishers.add(NetworkTableInstance.getDefault().getTable("Drive").getStructTopic("moduleRequest_" + i, SwerveModuleState.struct).publish());
        }

        wantedChassisSpeeds = new Parameter<ChassisSpeeds>(new ChassisSpeeds());
        wantedChassisSpeeds.onValueUpdated = (value) -> driveChassisSpeeds(value);
    }
    
    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.teleoperated);
            add(RunContext.autonomous);
        }};
    }

    public void onStart(RunContext ctx) {
        
    }
    //TODO: add a fieldSpeeds class
    private void driveChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState moduleStates[] = kinematics.toSwerveModuleStates(speeds);
        for (int i=0; i<modules.size(); i++) {
            modules.get(i).apply(SwerveModuleState.optimize(moduleStates[i], modules.get(i).getPosition(true).angle), DriveRequestType.Velocity);
            moduleRequestPublishers.get(i).set(SwerveModuleState.optimize(moduleStates[i], modules.get(i).getPosition(true).angle));
        }
    }

    public void onLoop(RunContext ctx) {

    }

    public void onStop() {
        driveChassisSpeeds(new ChassisSpeeds());
    }

    public void publishTelemetry() {
        for(int i=0; i<modules.size(); i++) {
            moduleStatePublishers.get(i).set(modules.get(i).getCurrentState());
        }
    }
}
