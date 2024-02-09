package frc.robot.Framework;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Set;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableType;

public class Config {
    NetworkTable configTable;
    boolean initialized = false;

    FileWriter file;
    FileReader reader;
    //Stream

    public double getDouble(String key, double safeDefault) {
        if(!initialized) return safeDefault;
        if(!configTable.getEntry(key).exists()) {
            configTable.getEntry(key).setDouble(safeDefault);
        }
        return configTable.getEntry(key).getDouble(safeDefault);
    }

    public void setDouble(String key, double value) {
        configTable.getEntry(key).setDouble(value);
    }

    public long getLong(String key, long safeDefault) {
        if(!initialized) return safeDefault;
        if(!configTable.getEntry(key).exists()) {
            configTable.getEntry(key).setInteger(safeDefault);
        }
        return configTable.getEntry(key).getInteger(safeDefault);
        
    }

    public void setLong(String key, long value) {
        configTable.getEntry(key).setInteger(value);
    }

    public void saveValues() {
        Set<String> doubleKeys = configTable.getKeys(NetworkTableType.kDouble.getValue());
        Set<String> longKeys = configTable.getKeys(NetworkTableType.kInteger.getValue());

        //write all values to the file

        try {
            for (String key : doubleKeys) {
                file.write("d:" + key + configTable.getEntry(key).getDouble(0) + "\n");
            }
            for (String key : longKeys) {
                file.write("i:" + key + configTable.getEntry(key).getInteger(0) + "\n");
            }
            System.out.println("saved all config values");
        } catch (IOException i) {
            System.out.println("Config: unable to write file");
            initialized = false;
        }
    }

    public void readValues() {
        
    }
}
