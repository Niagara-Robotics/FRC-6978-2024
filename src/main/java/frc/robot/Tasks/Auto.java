package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.AutoCommands.IntakeCommand;
import frc.robot.AutoCommands.LowerLauncherCommand;
import frc.robot.AutoCommands.RaiseLauncherCommand;
import frc.robot.AutoCommands.ShootCommand;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Subsystems;

public class Auto implements IPeriodicTask {
    
    Command testAuto;

    boolean isFinished = false;

    public Auto() {
        Subsystem dummySubsytem = new Subsystem() {
            
        };

        AutoBuilder.configureRamsete(
                () -> Subsystems.tracking.getPose(), // Robot pose supplier
                pose -> Subsystems.tracking.setOdometryPose(pose), // Method to reset odometry (will be called if your auto has a starting pose)
                () -> Subsystems.tracking.getChassisSpeeds(), // Current ChassisSpeeds supplier
                speeds -> Subsystems.differentialDrive.driveChassisSpeeds(speeds), // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE



                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        //return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                dummySubsytem // Reference to this subsystem to set requirements
        );

        IntakeCommand intakeCommand = new IntakeCommand();
        NamedCommands.registerCommand("Intake", intakeCommand);

        ShootCommand shootCommand = new ShootCommand();
        NamedCommands.registerCommand("Shoot", shootCommand);

        RaiseLauncherCommand raiseLauncherCommand = new RaiseLauncherCommand();
        NamedCommands.registerCommand("RaiseLauncher", raiseLauncherCommand);

        LowerLauncherCommand lowerLauncherCommand = new LowerLauncherCommand();
        NamedCommands.registerCommand("LowerLauncher", lowerLauncherCommand);

        testAuto = AutoBuilder.buildAuto("Red 2");
    }
    
    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.disabled);
            add(RunContext.teleoperated);
            add(RunContext.autonomous);
        }};
    }

    public void onStart(RunContext ctx) {
        testAuto.initialize();
    }

    public void onLoop(RunContext ctx) {
        if(isFinished) return;
        if(testAuto.isFinished()) {
            testAuto.end(false);
            this.isFinished = true;
        }
        testAuto.execute();
    }

    public void publishTelemetry() {}

    public void onStop() {
        testAuto.end(true);
        Subsystems.differentialDrive.coast();
    }
}
