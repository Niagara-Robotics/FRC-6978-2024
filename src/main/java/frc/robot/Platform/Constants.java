package frc.robot.Platform;

import java.util.HashMap;

public class Constants {
    public static class Drive {
        //volts

        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.000;
        public static double kVright = 0.109;
        public static double kSright = 0.04;
        public static double kVleft = 0.08;
        public static double kSleft = 0.04;

        public static double wheelRotorRatio = 0.1629;

        public static double peakOutput = 1;

        public static double xMultiplier = 0.4;
        public static double yMultiplier = 1;
        public static double maxX = 1;
        public static double maxY = 1;        
        public static double deadZone = 0.1;

        public static double maxVelocity = 20000;

        public static boolean gearShiftDefaultState = false;

        public static double balancer_yaw_kP = 0.001;
        public static double balancer_yaw_kI = 0.000;
        public static double balancer_yaw_kD = 0.000;

        public static double balancer_pitch_kP = 0.1;
        public static double balancer_pitch_kI = 0.0;
        public static double balancer_pitch_kD = 0.0;
    }
    
    public static class Arm {
        public static double slideStartingPosition = 0;
        public static double slideParkPosition = 0;
        public static double slideMaxExtension = 19000;
        public static double slide_kP = 0.15;
        public static double slide_kI = 0.0;
        public static double slide_kD = 30.0;

        public static HashMap<ArmPosition, Double> slidePositionMap = new HashMap<ArmPosition, Double>() {{
            put(ArmPosition.park, 0d);
            put(ArmPosition.partialPark, 0d);
            put(ArmPosition.groundPickup, 0d);
            put(ArmPosition.humanPickup, -0d);
            put(ArmPosition.cubeMid, 0d);
            put(ArmPosition.coneMid, 10051d);
            put(ArmPosition.cubeHigh, 17000d);
            put(ArmPosition.coneHigh, 19900d);
        }};
        
        public static double liftStartingPosition = 0;
        public static double liftLowerPosition = -800000;
        public static double liftUpperPosition = 0;
        public static double liftInsideBumperLower = -100000;
        public static double liftInsideBumperUpper = -5000;
        public static double lift_kP = 0.04;
        public static double lift_kI = 0.0;
        public static double lift_kD = 10;

        public static HashMap<ArmPosition, Double> liftPositionMap = new HashMap<ArmPosition, Double>() {{
            put(ArmPosition.park, 0d);
            put(ArmPosition.partialPark, 0d);
            put(ArmPosition.groundPickup, -640000d);
            put(ArmPosition.humanPickup, -370000d);
            put(ArmPosition.cubeMid, -531000d);
            put(ArmPosition.coneMid, -531000d);
            put(ArmPosition.cubeHigh, -358000d);
            put(ArmPosition.coneHigh, -358000d);
        }};

        public static double cableStartingPosition = 0;
        public static double cableBumperClearancePosition = 38000;
        public static double cableMaxPosition = 408000;
        public static double cable_kP = 0.01;
        public static double cable_kI = 0.0;
        public static double cable_kD = 0.0;

        public static HashMap<ArmPosition, Double> cablePositionMap = new HashMap<ArmPosition, Double>() {{
            put(ArmPosition.park, 0d);
            put(ArmPosition.partialPark, 37000d);
            put(ArmPosition.groundPickup, -76000d);
            put(ArmPosition.humanPickup, 362000d);
            put(ArmPosition.cubeMid, 226174d);
            put(ArmPosition.coneMid, 336000d);
            put(ArmPosition.cubeHigh, 370000d);
            put(ArmPosition.coneHigh, 402000d);
        }};

        public static boolean gripperCloseDefaultState = false;

        public static double wheelBias = .25;
        public static double wheelSlow = .25;
        public static double wheelFast = .8;
        public static double wheelLaunch = .7;
    }

    public static class DriverControls {
        public static int steeringAxis = 0;
        public static int forwardAxis = 4;
        public static int reverseAxis = 3;


        public static int gearShift = 10;
        public static int balance = 11;

        public static int slowWheelsIn = 1;
        public static int fastWheelsIn = 4;
        public static int fastWheelsOut = 3;
    }

    public static class OperatorControls {
        public static int zeroArm = 10;

        public static int toggleClaw = 4;//south

        public static int slideIn = 5;
        public static int slideOut = 6;
        
        public static int cableAxis = 1;
        public static int liftAxis = 3;

        public static int toggleArmAutomation = 9; 

        //auto mode buttons
        public static int park = 1; //west
        public static int partialParkPOV = 180; //pov hat
        public static int groundPickup = 3; //east
        public static int humanPickup = 2; //north
        public static int cubeMid = 5; //left bumper
        public static int coneMid = 6; //right bumper
        public static int cubeHigh = 7; //left trigger
        public static int coneHigh = 8; //right trigger
    }
}
