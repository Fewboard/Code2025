package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class Constants {
    public static final class ModuleConstants{
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
        public static final double kDriveMotorGearRatio = 1.0 / 6.75;
        public static final double kTurningMotorGearRatio = 1.0 / (150.0/7.0);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        public static final double kPTurning = 0.3;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = 0.56;
        // Distance between right and left wheels
        public static final double kWheelBase = 0.56;
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
                // Front Left 
                // Front Right
                // Back Left 
                // Back Right

        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kBackRightDriveMotorPort = 7;

        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kBackRightTurningMotorPort = 8;

        public static final int kElevatorMotorPort = 8;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final int kBackRightDriveAbsoluteEncoderPort = 12;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double kNoteCollectMaxSpeed = 2.5;

        public static double kSpeed = 1.5;
        public static double kSpeedAngular = 1.5;

        public static double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / kSpeed;
        public static double kTeleDriveMaxAngularSpeedRadiansPerSecond = // 
                kPhysicalMaxAngularSpeedRadiansPerSecond / kSpeedAngular;
        public static double kTeleDriveMaxAccelerationUnitsPerSecond = 1.5;
        public static double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;

        public static final double ksVolts = 0.29565;
        public static final double kvVoltSecondsPerMeter = 0.8224;
        public static final double kaVoltSecondsSquaredPerMeter = 1.4512;
        public static final double kPDriveVel = 1.1949;
    }

    public static final class FieldPoses {
        // Load the path you want to follow using its name in the GUI
        public static PathPlannerPath kCoralBlueLeftPath;
        public static PathPlannerPath kCoralBlueRightPath;

        public static final Pose2d kCoralBlueLeft = new Pose2d(1.11, 6.93, new Rotation2d(-1));
        public static final Pose2d kCoralBlueRight = new Pose2d(1.06, 0.96, new Rotation2d(1));
    
        // public static final Pose2d kGrid1 = new Pose2d(, , new Rotation2d(-1.14));
        // public static final Pose2d kGrid2 = new Pose2d(1.05, 7.10, new Rotation2d(-1));
        // public static final Pose2d kGrid3 = new Pose2d(1.05, 7.10, new Rotation2d(-1));
        // public static final Pose2d kGrid4 = new Pose2d(1.05, 7.10, new Rotation2d(-1));
        // public static final Pose2d kGrid5 = new Pose2d(1.05, 7.10, new Rotation2d(-1));
        // public static final Pose2d kGrid6 = new Pose2d(1.05, 7.10, new Rotation2d(-1));
        // public static final Pose2d kGrid7 = new Pose2d(1.05, 7.10, new Rotation2d(-1));
        // public static final Pose2d kGrid8 = new Pose2d(1.05, 7.10, new Rotation2d(-1));
        // public static final Pose2d kGrid9 = new Pose2d(1.05, 7.10, new Rotation2d(-1));
        // public static final Pose2d kGrid10 = new Pose2d(1.05, 7.10, new Rotation2d(-1));
        public static final Pose2d kReef6 = new Pose2d(3.48, 5.26, new Rotation2d(-1));
        public static final Pose2d kGrid11 = new Pose2d(3.86, 5.26, new Rotation2d(-1));
        public static final Pose2d kGrid12 = new Pose2d(3.53, 5.10, new Rotation2d(-1));

        public static final Pose2d[] gridPoses = {kGrid11, kGrid12};
    }

    public static final class elevatorConstants {
        public static final int elevatorMotorID = 13;
        
        public static final double CollectPosition = 4.1;
        public static final double L1Position = 4.1;
        public static final double L2Position = 38;
        public static final double L3Position = 88;
        public static final double L4Position = 88;

        public static final double elevatorMax = 88;
        public static final double elevatorMin = 2;
        public static double elevatorSetpoint = 0;
    }

    public static final class intakeConstants {
        public static final int intakeMotorID = 14;
        public static final int controlMotorID = 15;
        public static final int hoodMotorID = 16;
        public static final int CANrangeFirstID = 20;
        public static final int CANrangeSecondID = 21;

        public static final double kCANrangeDetectionLimit = 0.1;
        public static final double CollectPosition = 1;
        public static final double L1Position = 3.02;
        public static final double L2Position = 1.33;
        public static final double L3Position = 2.3;
        public static final double L4Position = 17.4;

        public static final double intakeMaxSpeed = 0.4;
        public static final double intakeMax = 24.2;
        public static final double intakeMin = 0;
        public static Boolean intakeCollecting = false;
        public static double intakeSetpoint = 0;
    }

    public static final class climbConstants {
        public static final int clawMotorID = 17;
        public static final int planetaryMotorID = 18;

        public static final double openPosition = 0;
        public static final double closedPosition = 0;

        public static final double climbMaxSpeed = 0.5;
        public static final double climbMinSpeed = 0.5;
        public static final double climbMax = 400;
        public static final double climbMin = -400;

        public static final double clawMax = 4;
        public static final double clawMin = 0;

        public static final double climbP = 0.7;
        public static final double clawP= 0.7;
        public static double climbSetpoint = 0;
        public static double clawSetpoint = 0;
    }

    public static final class AutoConstants {
        public static PathConstraints constraints = new PathConstraints(2.5, 1.5, 
        Units.degreesToRadians(270), Units.degreesToRadians(450));

    }
    
    public static final class LimelightConstants {
        public static String LimelightCoral = "limelight-coral"; 
        public static String LimelightReef = "limelight-reef"; 
    }

    public static final class XboxConstants {
        public static final int kResetEncodersButtonIdx = 3;
        
    }

    public static final class LogitechConstants {
        public static final int kResetEncodersButtonIdx = 1;

    }

    public static final class OIConstants {
        public static Boolean isXbox = DriverStation.getJoystickIsXbox(0);

        public static double getGyroAxis(Joystick joystick){
            return isXbox ? joystick.getRawAxis(4) : joystick.getZ();
        }

        // ============================ BOTÕES DO CONTROLE ===========================
        //Controle Swerve
        public static final int kResetEncodersButtonIdx = isXbox ? XboxConstants.kResetEncodersButtonIdx : LogitechConstants.kResetEncodersButtonIdx;

        //Controle Garra
        public static final int kIntakeInputButtonIdx = 5;
        public static final int kIntakeOutputButtonIdx = 6;
        public static final int kHoodInputButtonIdx = 7;
        public static final int kHoodOutputButtonIdx = 8;
        public static final int kClimbUpButtonIdx = 9;
        public static final int kClimbDownButtonIdx = 10;
        public static final int kClawOpenButtonIdx = 11;
        public static final int kClawCloseButtonIdx = 12;
        public static final int kRaiseIntakeButtonIdx = 90;
        public static final int kLowerIntakeButtonIdx = 270;

        public static final int kRaiseElevatorButtonIdx = 0;
        public static final int kLowerElevatorButtonIdx = 180;

        public static final int kCollectCoralButtonIdx = 3;
        public static final int kScoreCoralButtonIdx = 4;
        public static final int kCollectButtonIdx = 2;
        public static final int kL1ButtonIdx = 180;
        public static final int kL2ButtonIdx = 90;
        public static final int kL3ButtonIdx = 270;
        public static final int kL4ButtonIdx = 0;
        // ============================================================================

        public static final int kDriverControllerPort = 0;
        public static final int kSecondDriverControllerPort = 1;

        public static final int kDriverXAxis = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverRotAxis = 2;
        
        public static final double kDeadband = 0.05;   

    }

}

