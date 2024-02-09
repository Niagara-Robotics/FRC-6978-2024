package frc.robot.Platform;

public class Constants {
    public static class Drive {
        //volts
        public static double kP = 0.05;
        public static double kI = 0.0;
        public static double kD = 0.000;
        public static double kVright = 0.109;
        public static double kSright = 0.04;
        public static double kVleft = 0.1;
        public static double kSleft = 0.04;

        //TODO: update wheelRotorRatio
        public static double rotorToMeters = 0.039012;
        public static double trackWidth = 0.9; //meters

        public static double peakOutput = 1;

        public static double xMultiplier = 1;
        public static double yMultiplier = 4;
        public static double maxLinearVelocity = 4;
        public static double maxAngularVelocity = 1;
        public static double deadZone = 0.1;
    }

    public static class Launcher {
        public static double rotorToMeters = 5;
        //60,12
        //default stage 2 velocity target(this is also parameterized)
        public static double defaultVelocity = 90;
        public static double defaultSpinVelocity = 0;

        public static double stage1Voltage = 10;
        public static double stage1Velocity = 70;

        //+/- this value automatically starts the stage 1 rollers and the intake feeder(index roller)
        public static double stage2Tolerance = 2;

        public static double kP = 1.0;
        public static double left_kV = 7.04 / 63;
        public static double right_kV = 7.04 / 65;
        public static double kS = 0.15;

        public static double tiltRotorToMechanismRatio = 50;
        //both of these need to be POSITIVE!!
        public static double tiltMaxOutputUp = 1.0;
        public static double tiltMaxOutputDown = 0.2;
        public static double tiltKp = 70;
        public static double tiltKg = 0.25;

        public static double tiltDefaultPosition = 0;

        public static double tiltMaxPosition = 0.195;
        public static double tiltMinPosition = 0;
    }

    public static class Intake {
        public static double floorRollerPower = 0.8;
        public static double indexRollerPower = 0.35;
        public static double indexRollerFeedLauncherPower = 1;
    }

    public static class Lift {
        public static double power = 0.5;
    }

    public static class DriverControls {
        public static int steeringAxis = 0;
        public static int forwardAxis = 4;
        public static int reverseAxis = 3;

        public static int intakeButton = 3;
    }

    public static class OperatorControls {
        public static int launcherButton = 2;

        public static int climbUpButton = 1;
        public static int climbReleaseButton = 3;
    }
}
