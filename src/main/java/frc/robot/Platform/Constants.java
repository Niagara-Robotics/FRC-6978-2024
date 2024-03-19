package frc.robot.Platform;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static class Drive {
        //volts
        public static double kP = 0.2;
        public static double kI = 0.0;
        public static double kD = 0.000;
        public static double kVright = 0.109;
        public static double kSright = 0.19;
        public static double kVleft = 0.1085;
        public static double kSleft = 0.19;

        public static double supplyCurrentLimit = 50;

        public static double rotorToMeters = 0.04045525091; //25.266 rotations per meter 
        public static double trackWidth = 1.25; //meters

        //6.231m 158.5

        public static double peakOutput = 1;

        public static double xMultiplier = 0.8;
        public static double yMultiplier = 1.5;
        public static double maxLinearVelocity = 4;
        public static double maxAngularVelocity = 1;
        public static double deadZone = 0.1;
    }

    public static class Tracking {
        public static double zClamp = 620;
        public static double minCornerDist = 22;
    }

    public static class Launcher {
        public static double rotorToMeters = 5;

        public static double maxWheelCurrent = 15;

        //60,12
        //default stage 2 velocity target(this is also parameterized)
        public static double defaultVelocity = 80;
        public static double ampVelocity = 20;
        public static double defaultSpinVelocity = 0;

        public static double stage1Voltage = 10;
        public static double stage1Velocity = 70;

        //+/- this value automatically starts the stage 1 rollers and the intake feeder(index roller)
        public static double stage2Tolerance = 2;

        public static double kP = 0.3;
        public static double left_kV = 7.04 / 62;
        public static double right_kV = 7.04 / 64;
        public static double kS = 0.15;

        public static double tiltRotorToMechanismRatio = 50.0 / (2.0*Math.PI);
        //both of these need to be POSITIVE!!
        public static double tiltMaxOutputUp = 3.0;
        public static double tiltMaxOutputDown = 0.95;
        public static double tiltKp = 95; //TODO: estimate for appropriate kP with new sensor ratio
        public static double tiltKg = 0.313; //TODO: determine appropriate kG for cosine approximation

        public static double tiltDefaultPosition = -0.1134464014;

        public static double tiltMaxPosition = 1.19; //0.195rot TODO: determine max tilt position in radians
        public static double tiltMinPosition = -0.113446;

        public static double tiltTolerance = 0.0081;

        public static double ampTiltPosition = 1.0;

        public static double trapTiltPosition = 1.05;

        public static double trapVelocity = 70;

        
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
            put(1370.0, 60.0);
            put(1930.0, 70.0);
            put(2280.0, 80.0);
            put(2740.0, 80.0);
            put(3220.0, 100.0);
        }};

        //TODO: remap tilt position in radians
        public static HashMap<Double, Double> tiltMap = new HashMap<Double,Double>() { {
            put(1370.0, 0.19*2*Math.PI);
            put(1930.0, 0.13*2*Math.PI);
            put(2280.0, 0.09*2*Math.PI);
            put(2740.0, 0.35);
            put(3220.0, 0.029*2*Math.PI);
            put(3600.0, 0.0);
            put(4000.0, -0.1);
        }};

        public static HashMap<Integer, Pose2d> redTrapFiringPoints = new HashMap<Integer, Pose2d>() {{
            put(0, new Pose2d(11.905, 3.713, new Rotation2d(-1.047))); //stage left id 11
            put(1, new Pose2d(11.220, 4.105, new Rotation2d(3.141))); //center stage id 13
            put(2, new Pose2d(11.905, 4.498, new Rotation2d(1.047))); //stage right id 12
        }};

        public static HashMap<Integer, Pose2d> blueTrapFiringPoints = new HashMap<Integer, Pose2d>() {{
            put(0, new Pose2d(4.641, 4.498, new Rotation2d(2.094))); //stage left id 15
            put(1, new Pose2d(5.321, 4.105, new Rotation2d(0))); //center stage id 13
            put(2, new Pose2d(4.641, 3.713, new Rotation2d(-2.094))); //stage right id 16
        }};

        public static Pose2d redAmpPose = new Pose2d(14.7, 8.2, new Rotation2d(-1.57));
        public static Pose2d blueAmpPose = new Pose2d(1.84, 8.2, new Rotation2d(-1.57));
        public static double ampSecondWhiskerLength = 0.8;

        public static double ampWhiskerLength = 0.3;


        public static double trapWhiskerLength = 0.85;

        public static boolean dropTiltAfterSpeakerShot = true;
    }

    public static class Intake {
        public static double floorRollerPower = 0.55;
        public static double indexRollerVelocity = 7.5;

        public static double indexer_kV = 7.04 / 63;
        public static double indexer_kP = 0.45;
        public static double indexer_kS = 0.08;

        public static double indexRollerFeedLauncherPower = 10;
    }

    public static class Lift {
        public static double power = 0.5;
        public static double kP = 1.2;
        public static double kF = 1.4;
        public static double velocity = 240;
        public static double catchPoint = 7350;
    }

    public static class AutoPilot {
        public static double anglekP = 4.5;
        public static double angularVelocityLimit = 3.0; //radians per second

        public static double distancekP = 4.0;
        public static double linearVelocityLimit = 1.5; //radians per second
        public static double deltaStopPoint = 0.8;

        public static double distanceThreshold = 0.04;
    }

    public static class DriverControls {
        public static int steeringAxis = 0;
        public static int forwardAxis = 4;
        public static int reverseAxis = 3;

        public static int intakeButton = 3;

        public static int transferControlButton = 13;

        public static int autoAlignButton = 6;
        public static int autoNoteButton = 10;
        public static int ampShotButton = 5;
        public static int trapShotButton = 9;

        public static int tiltTakeover = 12;
        public static int generalAlignButton = 14;
    }

    public static class OperatorControls {
        public static int launcherButton = 2;

        public static int climbUpButton = 1;
        public static int climbReleaseButton = 3;

        public static int configureSubShotButton = 5;
        public static int configurePodiumShotButton = 6;
    }
}
