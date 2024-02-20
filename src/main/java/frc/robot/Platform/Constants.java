package frc.robot.Platform;

import java.util.HashMap;
import java.util.Map;

public class Constants {
    public static class Drive {
        //volts
        public static double kP = 0.05;
        public static double kI = 0.0;
        public static double kD = 0.000;
        public static double kVright = 0.109;
        public static double kSright = 0.08;
        public static double kVleft = 0.1085;
        public static double kSleft = 0.08;

        //TODO: update wheelRotorRatio
        public static double rotorToMeters = 0.039012;
        public static double trackWidth = 1.5; //meters

        public static double peakOutput = 1;

        public static double xMultiplier = 0.8;
        public static double yMultiplier = 1.5;
        public static double maxLinearVelocity = 4;
        public static double maxAngularVelocity = 1;
        public static double deadZone = 0.1;
    }

    public static class Launcher {
        public static double rotorToMeters = 5;
        //60,12
        //default stage 2 velocity target(this is also parameterized)
        public static double defaultVelocity = 100;
        public static double ampVelocity = 20;
        public static double defaultSpinVelocity = 0;

        public static double stage1Voltage = 10;
        public static double stage1Velocity = 70;

        //+/- this value automatically starts the stage 1 rollers and the intake feeder(index roller)
        public static double stage2Tolerance = 2;

        public static double kP = 0.3;
        public static double left_kV = 7.04 / 63;
        public static double right_kV = 7.04 / 65;
        public static double kS = 0.15;

        public static double tiltRotorToMechanismRatio = 50;
        //both of these need to be POSITIVE!!
        public static double tiltMaxOutputUp = 1.3;
        public static double tiltMaxOutputDown = 0.6;
        public static double tiltKp = 80;
        public static double tiltKg = 0.25;

        public static double tiltDefaultPosition = 0;

        public static double tiltMaxPosition = 0.195;
        public static double tiltMinPosition = 0;

        public static double tiltTolerance = 0.0055;

        public static double ampTiltPosition = .145;

        
    }

    public static class AutoShot {
        /*velocity,tilt map
        1370: 90,0.19
        1930: 90,0.13
        2280: 90, 0.09
        2740: 90, 0.06
        3220: 100, 0.023
        */
        public static HashMap<Double, Double> velocityMap = new HashMap<Double,Double>() { {
            put(1370.0, 90.0);
            put(1930.0, 90.0);
            put(2280.0, 90.0);
            put(2740.0, 90.0);
            put(3220.0, 100.0);
        }};

        public static HashMap<Double, Double> tiltMap = new HashMap<Double,Double>() { {
            put(1370.0, 0.19);
            put(1930.0, 0.13);
            put(2280.0, 0.09);
            put(2740.0, 0.06);
            put(3220.0, 0.023);
        }};
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

        public static int transferControlButton = 13;

        public static int autoAlignButton = 6;
        public static int ampShotButton = 5;

        public static int tiltTakeover = 12;
    }

    public static class OperatorControls {
        public static int launcherButton = 2;

        public static int climbUpButton = 1;
        public static int climbReleaseButton = 3;
    }
}
