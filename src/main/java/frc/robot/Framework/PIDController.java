package frc.robot.Framework;

public class PIDController {
    double kP, kI, kD;
    double target;
    double lastError;
    double integralAccumulator;
    boolean firstCycle;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void init() {
        firstCycle = true;
        integralAccumulator = 0;
    }

    public void set(double target) {
        this.target = target;
    }

    //return an output demand
    public double process(double sensorValue) {
        double error = target - sensorValue;
        double proportional, integral, derivative;

        integralAccumulator += error;

        proportional = error * kP;
        integral = integralAccumulator * kP;
        if(!firstCycle)  {
            derivative = (error - lastError);
        } else {
            derivative = 0;
            firstCycle = false;
        }
        lastError = error;

        return proportional + integral + derivative;
    }
}
