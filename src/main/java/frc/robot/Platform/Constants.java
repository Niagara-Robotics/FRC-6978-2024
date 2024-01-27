package frc.robot.Platform;

public class Constants {
    public static class Drive {
        //volts
        public static double kP = 0.1;
        public static double kI = 0.0;
        public static double kD = 0.000;
        public static double kVright = 0.109;
        public static double kSright = 0.04;
        public static double kVleft = 0.01;
        public static double kSleft = 0.04;

        //TODO: update wheelRotorRatio
        public static double rotorToMeters = 0.039012;
        public static double trackWidth = 0.9; //meters

        public static double peakOutput = 1;

        public static double xMultiplier = 0.5;
        public static double yMultiplier = 1;
        public static double maxLinearVelocity = 1;
        public static double maxAngularVelocity = 0.5;
        public static double deadZone = 0.1;
    }
    public static class Intake {
        public static double floorRollerPower = 0.6;
        public static double indexRollerPower = 0.2;
    }

    public static class DriverControls {
        public static int steeringAxis = 0;
        public static int forwardAxis = 4;
        public static int reverseAxis = 3;
    }

    public static class OperatorControls {
    }
}
