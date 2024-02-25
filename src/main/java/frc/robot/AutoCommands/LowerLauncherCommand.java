package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Framework.ParameterHandle;
import frc.robot.Platform.Subsystems;

public class LowerLauncherCommand extends Command {
    ParameterHandle<Double> positionHandle;

    @Override
    public void initialize() {
        positionHandle = Subsystems.launcher.tilt.getHandle("LowerCommand");
        positionHandle.takeControl(false);
        positionHandle.set(0.0);

    }

    @Override
    public boolean isFinished() {
        return Subsystems.launcher.tiltFinished();
    }
}
