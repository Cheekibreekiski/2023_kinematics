package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import java.util.Hashtable;

public final class Constants {
    public static final int intakeMotorID = 11; //TODO: assign correct values
    public static final int wristMotorID = 12;
    public static final int wristCanCoderID = 14;
    public static final int armCanCoderID = 13;
    public static final int wristSolenoidID = 0;
    public static final int pHubID = 1;
    public static final int compressorID = 1;
    public static final double stickDeadband = 0.05;
    public static final int Jake = 194; //Jake M is 194 cm tall

    public static final class PoseEstimation {
        public static ArrayList<Pose2d> scoringPositions = new ArrayList<Pose2d>() {{
            // add(
            //     new Pose2d(Units.inchesToMeters(20.19 + 22.0 * 0), 2.18, new Rotation2d()),
            //     new Pose2d(Units.inchesToMeters(20.19 + 22.0 * 1), 2.18,  new Rotation2d()),
            //     new Pose2d(Units.inchesToMeters(20.19 + 22.0 * 2), 2.18,  new Rotation2d()),
            //     new Pose2d(Units.inchesToMeters(20.19 + 22.0 * 3), 2.18,  new Rotation2d()),
            //     new Pose2d(Units.inchesToMeters(20.19 + 22.0 * 4), 2.18,  new Rotation2d()),
            //     new Pose2d(Units.inchesToMeters(20.19 + 22.0 * 5), 2.18,  new Rotation2d()),
            //     new Pose2d(Units.inchesToMeters(20.19 + 22.0 * 6), 2.18,  new Rotation2d()),
            //     new Pose2d(Units.inchesToMeters(20.19 + 22.0 * 7), 2.18,  new Rotation2d()),
            //     new Pose2d(Units.inchesToMeters(20.19 + 22.0 * 8), 2.18,  new Rotation2d())                
            // );
            add(new Pose2d(1.95, Units.inchesToMeters(20.19 + 22.0 * 0), new Rotation2d(Math.PI)));//0, 
            add(new Pose2d(1.95, Units.inchesToMeters(20.19 + 22.0 * 1), new Rotation2d(Math.PI)));//1, center of grid
            add(new Pose2d(1.95, Units.inchesToMeters(20.19 + 22.0 * 2), new Rotation2d(Math.PI)));//2
            add(new Pose2d(1.95, Units.inchesToMeters(20.19 + 22.0 * 3), new Rotation2d(Math.PI)));//3
            add(new Pose2d(1.95, Units.inchesToMeters(20.19 + 22.0 * 4), new Rotation2d(Math.PI)));//4, middle column of middle grid
            add(new Pose2d(1.95, Units.inchesToMeters(20.19 + 22.0 * 5), new Rotation2d(Math.PI)));//5
            add(new Pose2d(1.95, Units.inchesToMeters(20.19 + 22.0 * 6), new Rotation2d(Math.PI)));//6
            add(new Pose2d(1.95, Units.inchesToMeters(20.19 + 22.0 * 7), new Rotation2d(Math.PI)));//7, center of grid
            add(new Pose2d(1.95, Units.inchesToMeters(20.19 + 22.0 * 8), new Rotation2d(Math.PI)));//8, 
        }};

        public static final Pose2d hpStation = new Pose2d(15.61, 7.34, new Rotation2d());
        
        public static final Pose2d[] grid1 = // dirty side
        {
            scoringPositions.get(0),
            scoringPositions.get(1),
            scoringPositions.get(2)
        };
        public static final Pose2d[] grid2 = // center
        {
            scoringPositions.get(3),
            scoringPositions.get(4),
            scoringPositions.get(5)
        };

        public static final Pose2d[] grid3 = // clean
        {
            scoringPositions.get(6),
            scoringPositions.get(7),
            scoringPositions.get(8)
        };

        public static final Pose2d[] chargeStation = // add 0.4*sqrt2 for robot corner when turning in worst position
        {
            new Pose2d(), // towards midfield dirty
            new Pose2d(), // towards midfield clean
            new Pose2d(), // towards grid dirty
            new Pose2d() // towards grid clean
        };
        
        public static final double kPXController = 3; // 3
        public static final double kPYController = 3; // 3
        public static final double kPThetaController = 3.9; // 3.7
    

    }

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
        
        public static final String CANivore = "CANt_open_file";// name of the canivore

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.521; // 20.5 in -> meters
        public static final double wheelBase = 0.521; // meters
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1; //0.70; //TODO: This must be tuned to specific robot 0.45
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Chmaracterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.15565 / 12); // TUNED
        public static final double driveKV = (2.0206 / 12);
        public static final double driveKA = (0.94648 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = Math.PI; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot

            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(108.80859375);
            
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot

            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(213.310546875);

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot

            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(203.02734375);

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot

            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(52.20703125);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: Tune drivekP first with kPX, kPY, and kPTheta as 0. Then tune the others.
        public static final double kMaxSpeedMetersPerSecond = 4.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        // public static final double kPXController = 2.3;
        // public static final double kPYController = 2.3;
        // public static final double kPThetaController = 3.6;

        public static final double kPXController = 1; // 2.5
        public static final double kPYController = 1; // 2.5
        public static final double kPThetaController = 2; // 3.4

        public static HashMap<String, Command> eventMap = new HashMap<>();
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    
    public class BalancingConstants{
        public static final double kP = 0.2275;//TODO: tune // .225
        public static final double kI = 0;
        public static final double kD = 0.025;
        public static final double InversekP = 0.225;//TODO: tune
        public static final double InversekI = 0;
        public static final double InversekD = 0;
        public static final double kToleranceDegrees = 2.5;//acceptable absolute error in degrees
        public static final double kSetpoint = 0.0; // we want a pitch of 0 degrees
        public static final double kSensitivity = 20; // throttle(pid/sensitivity)*max speed = meters per second to drive    
    }
    
    public static class ButtonHashtable {
        //hashtable used here to make use of key - value pairs
        //access through table.get(key);
        //add or change through table.put(key, value);
        //remove through table.remove(key);

        //If the types specified in the <>'s of the "New Hashtable<>" are the same as the first, you don't have to include them
        //Be sure to use WRAPPER CLASSES for primitive types
        public Hashtable<String, Integer> buttons = new Hashtable<>();
        public ButtonHashtable () {
            buttons.put("X_Button", 1);
            buttons.put("A_Button", 2);
            buttons.put("B_Button", 3);
            buttons.put("Y_Button", 4);
            
            buttons.put("Left_Bumper_Button", 5);
            buttons.put("Right_Bumper_Button", 6);
            buttons.put("Left_Trigger_Button", 7);
            buttons.put("Right_Trigger_Button", 8);

            //for some reason there are some variables in 2022 rapid react also
            //with 1 and 0

            buttons.put("D_Pad_Up", 0);
            buttons.put("D_Pad_Right", 90);
            buttons.put("D_Pad_Down", 180);
            buttons.put("D_Pad_Left", 2700);
        }
        
    }   

    public static final class FieldConstants {
        public static final double length = 16.54175;
        public static final double width = 8.0137;
    }

    public static final class ArmConstants {
        public static final int armMotorPort = 10; 
        public static final int stowedLimitSwitch = 2;

        public static final double armOffset = 209; // 5 degrees at start // 220
        public static final double startingArmPos = 10; // in box //was 10
        public static final double groundPickupArm = 80; // ground intake // 82.5
        public static final double chuteArmPos = 30; // pickup from hp chute //was 35
        public static final double shelfArmPos = 17; // 13
        public static final double autoCarry = 38.5;
        public static final double trueArmMaxExtension = 200; // TODO: change!
        public static final double minNonCollidingExtention = 28; // limit for arm so it doesn't crash into the elevator // 32
        public static final double midScoringPos = 25.65; // 28.65
        public static final double highScoringPos = 50.21; // 57.21
        public static final double topSuck = 70.9;

        public static int armState = 0;
    }

    public static final class ElevatorConstants {
        public static final int elevatorMotorPort = 9;
        public static final int topLimitPort = 1;
        public static final int bottomLimitPort = 0;

        public static final int bottomPos = 0;
        public static final int midPos = -62256;
        public static final int highPos = -104446;
        public static final int shelfPos = -76050;

        // public static final int pos0 = 20; // done
        // public static final int pos1 = -65138; // dpne; old = -126256; 
        // public static final int pos2 = -107809; // done; old = -226710; 
        // public static final double topLimit = -236710;

        public static int elevatorState = 0;
    }
    
    public static final class IntakeConstants {
        public static final double gamepieceThreshold = 100.0; // placeholder value
    }

    public static final class LEDConstants {
        public static final int ledPort = 0; // placeholder
        // For use when leds are connected to the rio with pwm
        public static final int pwmLedPort1 = 99; // placeholder value
        public static final int pwmLedPort2 = 99; // placeholder value
    }

    public static final class WristConstants {
        public static final double wristOffset = 163.66; // 5 degrees at start
        public static final double globalWristMaxAngleUp = 250; // chute wrist position as well 260
        public static final double babyBirdPos = 255;
        public static final double globalWristMaxAngleDown = 352; 
        public static final double midScorePos = 345.76;
        public static final double groundPickupWrist = 275; // ground intake
        public static final double startingPosWrist = 320; // in box
        public static final double highScorePos = 304.6;
        public static final double shelfPickupPos = 359;
        public static final double topSuck = 296;
        public static final double autoCarry = 322;
        public static final double wristConditionForElevatorMovement = 263.58;
        public static final double cubeLaunch = 266;
    }  

}