package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Platform.Subsystems;

public class RaiseLauncherCommand extends Command {
    @Override
    public void initialize() {
        Subsystems.launcher.setTiltPosition(0.19);
    }

    @Override
    public boolean isFinished() {
        return Subsystems.launcher.tiltFinished();
    }
}
