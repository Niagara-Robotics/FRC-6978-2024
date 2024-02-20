package frc.robot.Framework;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Platform.Subsystems;

public class Scheduler implements Runnable{
    List<IPeriodicTask> tasks = new ArrayList<IPeriodicTask>();

    String name;

    RunContext context;

    boolean telemetry;

    boolean exitFlag;

    public Scheduler(String name, boolean tele, RunContext context) {
        this.name = name;
        telemetry = tele;
        this.context = context;
    }

    public void add(IPeriodicTask task) {
        tasks.add(task);
        if(!telemetry) task.onStart(context);
    }

    public void remove(IPeriodicTask task) {
        tasks.remove(task);
    }

    public void clear() {
        exitFlag = true;
        for (IPeriodicTask task : tasks) {
                    task.onStop();
                }
        tasks.clear();
        exitFlag = false;
    }

    public void run() {
        //System.out.println("[" + name + " scheduler] starting");
        exitFlag = false;

        if(tasks.isEmpty()) return;
        double totalDelta = 0;
        for (IPeriodicTask task : tasks) {
            if(task.getAllowedRunContexts().contains(context) || telemetry) {
                try{
                    long startTS = System.nanoTime();
                    if(telemetry) {task.publishTelemetry();} else {
                    task.onLoop(context);
                    }

                    double delta = (double)(System.nanoTime() - startTS) / 1000000.0;
                    totalDelta += delta;
                    if(!telemetry) Subsystems.telemetry.pushDouble(task.getClass().getName()+"_delta_T", delta); else
                    Subsystems.telemetry.pushDouble(task.getClass().getName()+"_telemetry_delta_T", delta);
                }
                catch (Exception e) {
                    System.out.println("[" + name + 
                    " scheduler]" + task.getClass() + " encountered exception " + e.getMessage());
                }
            }
        }
        Subsystems.telemetry.pushDouble("scheduler." + name + ".delta_T", totalDelta);

        /*try {
            sleep(5);
        } catch (InterruptedException e) {
            break;
        }*/
        
        //this.run()
        //System.out.println("[" + name + " scheduler] exiting");
    }
}
