package frc.robot.Framework;

import java.util.List;

public interface IPeriodicTask {
    //Called once every time this task enters context
    public void onStart(RunContext context);
    
    //Called once every time this task leaves context
    public void onStop();

    //Called once each frame while this task is in context
    public void onLoop(RunContext context);

    //Used to retrieve a list of RunContexts that this task may run inside of
    public List<RunContext> getAllowedRunContexts();
}
