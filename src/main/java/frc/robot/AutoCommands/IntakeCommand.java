package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Platform.Subsystems;

public class IntakeCommand extends Command {
    @Override
    public void initialize() {
        Subsystems.intake.intakeNote();
    }

    @Override
    public boolean isFinished() {
        return Subsystems.intake.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted) return; //dont call endIntake because it ends itself
        Subsystems.intake.idleIntake();
    }
}
