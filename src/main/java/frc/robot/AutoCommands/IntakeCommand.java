package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Platform.Subsystems;

public class IntakeCommand extends Command {
    Timer timer;

    @Override
    public void initialize() {
        Subsystems.intake.intakeNote();
        timer = new Timer();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        //return Subsystems.intake.hasNote();
        return timer.get() > 3.0;
    }

    @Override
    public void end(boolean interrupted) {
        //if(!interrupted) return; //dont call endIntake because it ends itself
        Subsystems.intake.idleIntake();
    }
}
