package frc.robot.Framework;

public class JoystickHelpers {
    public static double deadZone(double input, double deadzone) {

        input = (Math.abs(input) < deadzone)? 0 : input;
        input = (input > deadzone)? ((input-deadzone)/(1-deadzone)) : input;
        input = (input < -deadzone)? ((input+deadzone)/(1-deadzone)) : input;
        
        return input;
    }
}
