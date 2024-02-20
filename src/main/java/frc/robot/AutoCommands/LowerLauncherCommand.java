package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Platform.Subsystems;

public class LowerLauncherCommand extends Command {
    @Override
    public void initialize() {
        Subsystems.launcher.setTiltPosition(0);
    }

    @Override
    public boolean isFinished() {
        return Subsystems.launcher.tiltFinished();
    }
}
