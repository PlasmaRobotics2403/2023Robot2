package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final int DRIVER_JOYSTICK_PORT = 0;
    public static final int COPILOT_JOYSTICK_PORT = 1;

    public static final int TIMEOUT_MS = 60;

    public static final double CURRENT_LIMIT = 30;
    public static final double CURRENT_THRESHOLD = 30;
    public static final double THRESHOLD_TIME = 0;

    public static final class Swerve {
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(23); //TODO: This must be tuned to specific robot
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
        public static final double openLoopRamp = 0.5;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKF = 0.035;
        public static final double driveKP = 0.03; //TODO: This must be tuned to specific robot was 0.2214
        public static final double driveKI = 0;
        public static final double driveKD = 0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.16215 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (0.73438 / 12);
        public static final double driveKA = (0.11115 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 10; //TODO: This must be tuned to specific robot
        public static final double creepSpeed = 0.25;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /*Balance Multiplier */
        public static final double balanceMultiplier = 0.0142;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(70.14);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(315.26);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(290.21);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(262.31);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class ElevatorConstants {
        public static int master = 8;
        public static int slave = 9;

        public static double elevatorSpeed = 0.3;
        public static double ticksToInch = 0.000811297;
        public static double inchesToTicks = 1232.5937;

        public static final int ELEVATOR_LIMIT_ID = 0;
        public static final double MAX_EXTEND = 65000;

        public static final double ELEVATOR_HIGH_EXTEND = 65000;
        public static final double ELEVATOR_FEEDER_EXTEND = 10000;
        public static final double ELEVATOR_MID_EXTEND = 47000;
        public static final double ELEVATOR_LOW_EXTEND = 20000;
        public static final double ELEVATOR_BOTTTOM_EXTEND = 0;

        /*Elevator Pid */
        public static final int PID_IDX = 0;
        public static final int SLOT_IDX = 0;

        public static final double MOTION_CRUISE_VELOCITY = 5000;
        public static final double MOTION_ACCELERATION = 1000;

        public static final double kF = 0.1;
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class GrabberConstants {
        public static boolean cancoder_invert = true;

        public static final int ARM_ID = 10;
        public static final int GRABBER_ID = 11;
        public static final int GRABBER_CC_ID = 0;
        public static final int GRABBER_LIMITSWITCH = 1;
        public static final double grabberCCoffset = 52.85;

        public static final double ARM_HIGH_EXTEND = -28;
        public static final double ARM_MID_EXTEND = -56;
        public static final double ARM_LOW_EXTEND = 0;
        public static final double ARM_STOW_EXTEND = 0;
        public static final double ARM_CONE_FEEDER_EXTEND = 0;
        public static final double ARM_CUBE_FEEDER_EXTEND = 52;

        public static final double armkF = 0.0;
        public static final double armkP = 0.0032;
        public static final double armkI = 0;
        public static final double armkD = 0;
    }

   
    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 0.3;
        public static final double kPYController = 0.3;
        public static final double kPThetaController = -3;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
