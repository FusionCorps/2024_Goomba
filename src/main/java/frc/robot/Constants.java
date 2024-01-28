package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Constants {

  public class TarsArmConstants {
    public static final double ARM_POS_1 = 25; // 25;
    public static final double ARM_POS_2 = -25; // -25;
  }

  public class AimingPIDS {

    // constants for PIDS in RotateToAngle, MoveToTarget, and AimToTarget
    public static final double toTargetRotKP = 0.8;
    public static final double toTargetRotKI = 0;
    public static final double toTargetRotKD = 0.04;

    public static final double toAngleRotKP = 4.25;
    public static final double toAngleRotKI = 0.0;
    public static final double toAngleRotKD = 0.0;

    public static final double strKP = 0.25;
    public static final double strKI = 0;
    public static final double strKD = 0.0;
  }

  public class DrivetrainConstants {
    static class CustomSlotGains extends Slot0Configs {
      public CustomSlotGains(double kP, double kI, double kD, double kV, double kS) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kS = kS;
      }
    }

    // Note: gains should be really small for swerve
    // private static final CustomSlotGains steerGains = new CustomSlotGains(0.3, 0, 0.0002, 0, 0);
    // private static final CustomSlotGains driveGains = new CustomSlotGains(0.14, 0, 0, 0, 0);
    private static final CustomSlotGains steerGains = new CustomSlotGains(100, 0, 0, 0, 0);
    private static final CustomSlotGains driveGains = new CustomSlotGains(5, 0, 0, 0, 0);

    // pathplanner pids
    public static final PIDConstants AUTO_TRANS_PID = new PIDConstants(10, 0, 0.2);
    public static final PIDConstants AUTO_ROT_PID = new PIDConstants(8, 0, 0.15);

    private static final double kCoupleRatio = 0.0;

    private static final double kDriveGearRatio = 6.75;
    private static final double kSteerGearRatio = 12.8;
    private static final double kWheelRadiusInches = 2;
    private static final int kPigeonId = 0;
    private static final boolean kSteerMotorReversed = false;
    private static final String kCANbusName = "";
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static double kSteerInertia = 0.00001;
    private static double kDriveInertia = 0.001;

    private static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId).withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator =
        new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(800)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSpeedAt12VoltsMps(
                6) // Theoretical free speed is 10 meters per second at 12v applied output
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
            .withCouplingGearRatio(
                kCoupleRatio) // Every 1 rotation of the azimuth results in couple ratio drive turns
            .withSteerMotorInverted(kSteerMotorReversed);

    private static final int kFrontLeftDriveMotorId = 8;
    private static final int kFrontLeftSteerMotorId = 0;
    private static final int kFrontLeftEncoderId = 12;
    private static final double kFrontLeftEncoderOffset = -0.217041;

    private static final double kFrontLeftXPosInches = 9.75;
    private static final double kFrontLeftYPosInches = 9.75;
    private static final int kFrontRightDriveMotorId = 4;
    private static final int kFrontRightSteerMotorId = 3;
    private static final int kFrontRightEncoderId = 13;
    private static final double kFrontRightEncoderOffset = -0.046142578125;

    private static final double kFrontRightXPosInches = 9.75;
    private static final double kFrontRightYPosInches = -9.75;
    private static final int kBackLeftDriveMotorId = 9;
    private static final int kBackLeftSteerMotorId = 1;
    private static final int kBackLeftEncoderId = 2;
    private static final double kBackLeftEncoderOffset = -0.864013671875;

    private static final double kBackLeftXPosInches = -9.75;
    private static final double kBackLeftYPosInches = 9.75;
    private static final int kBackRightDriveMotorId = 5;
    private static final int kBackRightSteerMotorId = 7;
    private static final int kBackRightEncoderId = 11;
    private static final double kBackRightEncoderOffset = -0.8466796875;

    private static final double kBackRightXPosInches = -9.75;
    private static final double kBackRightYPosInches = -9.75;

    private static final SwerveModuleConstants FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId,
            kFrontLeftDriveMotorId,
            kFrontLeftEncoderId,
            kFrontLeftEncoderOffset,
            Units.inchesToMeters(kFrontLeftXPosInches),
            Units.inchesToMeters(kFrontLeftYPosInches),
            kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId,
            kFrontRightDriveMotorId,
            kFrontRightEncoderId,
            kFrontRightEncoderOffset,
            Units.inchesToMeters(kFrontRightXPosInches),
            Units.inchesToMeters(kFrontRightYPosInches),
            kInvertRightSide);
    private static final SwerveModuleConstants BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId,
            kBackLeftDriveMotorId,
            kBackLeftEncoderId,
            kBackLeftEncoderOffset,
            Units.inchesToMeters(kBackLeftXPosInches),
            Units.inchesToMeters(kBackLeftYPosInches),
            kInvertLeftSide);
    private static final SwerveModuleConstants BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId,
            kBackRightDriveMotorId,
            kBackRightEncoderId,
            kBackRightEncoderOffset,
            Units.inchesToMeters(kBackRightXPosInches),
            Units.inchesToMeters(kBackRightYPosInches),
            kInvertRightSide);

    public static final CommandSwerveDrivetrain DriveTrain =
        new CommandSwerveDrivetrain(
            new Cameras(), DrivetrainConstants, 250, FrontLeft, FrontRight, BackLeft, BackRight);

    public static final double MaxSpeed = 5; // 5 meters per second max speed
    public static final double MaxAngularRate =
        2 * Math.PI; // 1 rotation per second max rotation rate

    public static final double FULL_LENGTH = 0.81; // in meters, includes bumpers
    public static final double FULL_WIDTH = 0.81; // in meters, includes bumpers

    public static final double FL_BL_DISTANCE = 19.5; // in inches
    public static final double FR_BR_DISTANCE = 19.5; // in inches
    public static final double DRIVEBASE_RADIUS =
        Units.inchesToMeters(
            Math.sqrt(
                Math.pow(FL_BL_DISTANCE / 2, 2)
                    + Math.pow(
                        FR_BR_DISTANCE / 2,
                        2))); // in METERS, distance from center of robot to module
  }

  public static final int INTAKE_MOTOR_ID = 25; // TODO: change this id
  public static final int TOP_SHOOTER_MOTOR_ID = 3; // TODO: change this id
  public static final int BOTTOM_SHOOTER_MOTOR_ID = 2; // TODO: change this id
  public static final int SHOOTER_PIVOT_MOTOR_ID = -1; // TODO: change this id

  public static final double PIVOT_kP = 0.2;
  public static final double PIVOT_kD = 0;
  public static final double PIVOT_kI = 0;

  public static final double PIVOT_START_POS = Units.degreesToRotations(30);
  public static final double PIVOT_CLIMB_UP_POS = Units.degreesToRotations(120);
  public static final double PIVOT_CLIMB_DOWN_POS = Units.degreesToRotations(40);

  public static final String LIMELIGHT_NAME = "limelight";
  public static final double LIMELIGHT_TX_RANGE_DEG = 29.8; // range is thus -29.8 to 29.8
  public static final double LIMELIGHT_TY_RANGE_DEG = 24.85; // range is thus -24.85 to 24.85

  public static final double camHeightMeters = Units.inchesToMeters(54); // dont change
  public static final double cameraPitchRadians = Units.degreesToRadians(1.0); // dont change
  // 48 1/8 to bottom (white boundary edge) of target, 5 1/4 to center of target
  public static final double sourceTargetHeightMeters = Units.inchesToMeters(53.375);
  // 51 7/8 to bottom of target, 5 1/4 to center of target
  public static final double speakerTargetHeightMeters = Units.inchesToMeters(57.125);
  // 48 1/8 to bottom (white boundary edge) of target, 5 1/4 to center of target
  public static final double ampTargetHeightMeters =
      Units.inchesToMeters(53.375); // identical height to source
  // 47.5 inches to bottom of target, 4.5 to center of target
  public static final double trapTargetHeightMeters = Units.inchesToMeters(52);

  public static DriverStation.Alliance allianceColor = null; // null by default, set in Robot.java
  public static int allianceLocation = -1; // -1 by default, set in Robot.java\

  public static ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");
}
