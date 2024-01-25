package frc.robot.Tasks;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.FileSystems;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;

import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Framework.RunContext;

public class Telemetry {
    HashMap<String,Object> currentFrame = new HashMap<String, Object>();
    HashMap<String,Long> events = new HashMap<String,Long>();
    long lastFrameSaveDuration, frameOpenTS;
    FileWriter file;
    int numFramesSaved = 0;

    boolean frameOpen, sessionOpen;

    public void openSession() {
        String eventName = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("EventName").getString("UnknownEvent");
        long fmsControlInfo = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("FMSControlData").getInteger(0);
        long matchNumber = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("MatchNumber").getInteger(0);

        String fileName;

        DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yy-MM-dd-hh-mm-ss");
        LocalDateTime now = LocalDateTime.now();

        //if fms connected
        if((fmsControlInfo & 16l) > 0) {
            fileName = eventName + "-" + matchNumber + "-" + dtf.format(now) + ".json";
        } else {
            fileName = "Testing-" + dtf.format(now) + ".json";
        }

        try {
            file = new FileWriter(FileSystems.getDefault().getPath(determineStoragePathLocation(), fileName).toFile());
            file.write("{\"frames\": [");
        } catch (IOException e) {
            System.out.println("Unable to open file " + determineStoragePathLocation() + fileName);
            return;
        }
        sessionOpen = true;
        numFramesSaved = 0;
    }

    public void pushDouble(String key, double value) {
        SmartDashboard.putNumber(key, value);
        currentFrame.put(key, (Double)value);
    }

    public void pushBoolean(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
        currentFrame.put(key, (Boolean)value);
    }

    public void pushString(String key, String value) {
        SmartDashboard.putString(key, value);
        currentFrame.put(key, (String)value);
    }

    public void pushEvent(String name) {
        events.put(name, System.nanoTime());
        System.out.println("[telemetry] " + name);
    }

    public void openFrame() {
        //if(frameOpen) System.out.println("WARNING: telemetry frame already open");
        frameOpenTS = System.nanoTime();
        currentFrame.clear();
        events.clear();
        frameOpen = true;
    }

    public void closeFrame() {
        long saveStartTS = System.nanoTime();
        try {
            if(numFramesSaved < 1) {
                file.write("{");
            } else {
                file.write(",{");
            }
            
            for(Entry<String, Object> entry : currentFrame.entrySet()) {
                file.write("\"" + (String)entry.getKey() + "\": ");
                Object value = entry.getValue();
                if(
                    value.getClass().equals(Double.class) || 
                    value.getClass().equals(Boolean.class)
                ){
                    file.write(value.toString() + ",");
                } else {
                    file.write("\"" + value.toString() + "\",");
                }
            }

            file.write("\"events\": [");
            boolean firstEntry = true;
            for(Entry<String, Long> entry : events.entrySet()) {
                if(firstEntry) {
                    firstEntry = false;
                } else {
                    file.write(",");
                }
                file.write("{\"" + (String)entry.getKey() + "\": ");
                file.write(entry.getValue().toString() + "}");
            }

            file.write("],");

            file.write("\"lastDuration\": " + ((Long)lastFrameSaveDuration).toString() + ",");
            file.write("\"frameOpenTS\": " + ((Long)frameOpenTS).toString() + ",");
            file.write("\"frameCloseTS\": " + ((Long)saveStartTS).toString());
            
            file.write("}");
            file.flush();

        } catch (IOException e){
            System.out.println("Encountered IOException while trying to write telemetry: " + e.toString());
        } 
        lastFrameSaveDuration = System.nanoTime() - saveStartTS;
        currentFrame.clear();
        events.clear();
        frameOpen = false;
        numFramesSaved++;
    }

    public void closeSession() {
        try{
            file.write("]}");
            file.flush();
            file.close();
        } catch (IOException e) {
            System.out.println("Unable to close file");
        }
        sessionOpen = false;
    }

    String determineStoragePathLocation() {
        return "~/telemetry";
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.autonomous);
            add(RunContext.teleoperated);
        }};
    }
}
