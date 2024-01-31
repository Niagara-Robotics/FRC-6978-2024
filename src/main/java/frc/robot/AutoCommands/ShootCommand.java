package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Platform.Subsystems;

public class ShootCommand extends Command {
    @Override
    public void initialize() {
        Subsystems.launcher.launchNote();
    }

    @Override
    public boolean isFinished() {
        return Subsystems.launcher.finished();
    }

    @Override
    public void end(boolean interrupted) {
        //if(!interrupted) return; //dont call endIntake because it ends itself
        Subsystems.launcher.stopLauncher();
    }
}
