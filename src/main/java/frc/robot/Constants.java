package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Drivetrain;

public class Constants {
  public class TarsArmConstants {
    public static final double ARM_POS_1 = 25; // 25;
    public static final double ARM_POS_2 = -25; // -25;
  }

  public class AimingPIDS {

    // TODO: tune these better
    // constants for PIDS in RotateToAngle, MoveToTarget, and AimToTarget
    public static final double toTargetRotKP = 0.32;
    public static final double toTargetRotKI = 0;
    public static final double toTargetRotKD = 0.001;

    public static final double toAngleRotKP = 0.25;
    public static final double toAngleRotKI = 0.0;
    public static final double toAngleRotKD = 0.0;

    public static final double strKP = 0.25;
    public static final double strKI = 0;
    public static final double strKD = 0.0;
  }

  public class CobraConstants {
    public static final double cbrKP = 0.1;
    public static final double cbrKI = 0;
    public static final double cbrKD = 0.1;
    public static final int cbrMotorID = -1;
  }

  public class StageAlignment {
    public static final double toleranceDeg = 1.5;
    public static final double runTime = 2;
  }

  public class DrivetrainConstants {
    // TODO: slip current, couple ratio

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains =
        new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);

    private static final Slot0Configs driveGains =
        new Slot0Configs()
            .withKP(0.18003)
            .withKI(0)
            .withKD(0.003)
            .withKS(0.22875)
            .withKV(0.1115)
            .withKA(0.016661);
    // pathplanner pids
    public static final PIDConstants AUTO_DRIVE_PID = new PIDConstants(11.8, 0.8, 0.05);
    public static final PIDConstants AUTO_STEER_PID = new PIDConstants(8, 0, 0.15);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output
    public static final double kSpeedAt12VoltsMps = 6.0046;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3;

    private static final double kDriveGearRatio = 5.142857142857142;
    private static final double kSteerGearRatio = 12.8;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = false;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "canivore";
    private static final int kPigeonId = 0;

    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId).withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator =
        new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 10;
    private static final int kFrontLeftSteerMotorId = 11;
    private static final int kFrontLeftEncoderId = 12;
    private static final double kFrontLeftEncoderOffset = 0.380615234375;

    private static final double kFrontLeftXPosInches = 9.75;
    private static final double kFrontLeftYPosInches = 9.75;

    // Front Right
    private static final int kFrontRightDriveMotorId = 13;
    private static final int kFrontRightSteerMotorId = 14;
    private static final int kFrontRightEncoderId = 15;
    private static final double kFrontRightEncoderOffset = 0.464599609375;

    private static final double kFrontRightXPosInches = 9.75;
    private static final double kFrontRightYPosInches = -9.75;

    // Back Left
    private static final int kBackLeftDriveMotorId = 16;
    private static final int kBackLeftSteerMotorId = 17;
    private static final int kBackLeftEncoderId = 18;
    private static final double kBackLeftEncoderOffset = 0.45263671875;

    private static final double kBackLeftXPosInches = -9.75;
    private static final double kBackLeftYPosInches = 9.75;

    // Back Right
    private static final int kBackRightDriveMotorId = 19;
    private static final int kBackRightSteerMotorId = 20;
    private static final int kBackRightEncoderId = 21;
    private static final double kBackRightEncoderOffset = 0.367431640625;

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

    public static final Drivetrain DriveTrain =
        new Drivetrain(
            new Cameras(), DrivetrainConstants, 250, FrontLeft, FrontRight, BackLeft, BackRight);

    public static final double MaxSpeed = 5; // 5 meters per second max speed
    public static final double MaxAngularRate =
        2 * Math.PI; // 1 rotation per second max rotation rate
    public static final double AimingDamper = 0.2;
    public static final double DriveDeadband = 0.05 * MaxSpeed;

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

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 6;
    

    public static final double INTAKE_RUN_PCT = 0.9;
  }

  public static class TransferHookConstants {
    public static final int TRANSFER_HOOK_MOTOR_ID = 25;

    public static final double TRANSFER_HOOK_kV = 0;
    public static final double TRANSFER_HOOK_kP = 3;
    public static final double TRANSFER_HOOK_kD = 0.01;
    public static final double TRANSFER_HOOK_kI = 0;

    public static final double TRANSFER_HOOK_POS_CLIMB = 67.48828125;

    // motion magic constraints
    public static final double TRANSFER_HOOK_CRUISE_VELOCITY = 100;
    public static final double TRANSFER_HOOK_ACCELERATION = 30;
    public static final double TRANSFER_HOOK_JERK = 30;

    public static final double TRANSFER_HOOK_ERROR = 0.5;
  }

  public static class PivotConstants {

    public static boolean IS_SHUTTLING = false;
    public static final double PIVOT_GEAR_RATIO = 31.25;
    // public static final double PIVOT_GEAR_RATIO =
    // (4.0)
    // * (50.0 / 24.0)
    // * (60.0 / 16.0); // multiply ratios of two gearboxes and then small gear
    // turning big gear
    // public static final double PIVOT_GEAR_RATIO =
    // (4.0)
    // * (50.0 / 24.0)
    // * (60.0 / 16.0)
    // * (66.0 / 9.0); // multiply ratios of two gearboxes and then small gear
    // turning big
    // gear
    // public static final double PIVOT_GEAR_RATIO =
    // 31.25; // multiply ratios of two gearboxes and then small gear turning big
    // gear

    // error threshold of the pivot (1 deg)
    // public static final double PIVOT_ERROR_THRESHOLD = 1 / 360 *
    // PIVOT_GEAR_RATIO;
    public static final double PIVOT_ERROR_THRESHOLD = 0.2;

    public static final int PIVOT_MOTOR_ID = 1;
    public static final int PIVOT_FOLLOWER_MOTOR_ID = 5;

    public static final double PIVOT_OFFSET = 0.307506;

    public static final double PIVOT_STOW_POS = 15.8;

    public static final double PIVOT_kV = 0;
    public static final double PIVOT_kP = 8;
    public static final double PIVOT_kD = 0.01;
    public static final double PIVOT_kI = 0;

    // motion magic constraints
    public static final double PIVOT_CRUISE_VELOCITY = 2000;
    public static final double PIVOT_ACCELERATION = 700;
    public static final double PIVOT_JERK = 6000;

    public static final double PIVOT_AMP_POS = -55.894775390625; // empirical testing

    public static final double PIVOT_SUB_POS = 31.9052734375;

    public static final double PIVOT_SHUTTLING_POS = 22.146728515625;

    public static final double PIVOT_READY_CLIMB_POS = 7.34228515625;

    public static final double PIVOT_TRAP_POS = -63.71630859375;

    // TODO: Change these values
    public static final double PIVOT_CLIMB_UP_POS = -12.61279296875;
    public static final double PIVOT_CLIMB_DOWN_POS = 35.70751953125;

    // maps Z distances to april tag (meters) with pivot angles (rotations)
    public static final InterpolatingDoubleTreeMap PIVOT_ANGLES_MAP =
        new InterpolatingDoubleTreeMap();
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_TOP_ID = 2;
    public static final int SHOOTER_MOTOR_BOTTOM_ID = 3;

    public static boolean IS_SHOOTING_RIGHT = false;

    public static boolean HAS_STOPPED_REVVING = false;
    public static boolean IS_AMP = false;

    public static final double ShooterSpeed = 28.06308713961776; // in ft/s

    // the speeds of the shooter motors when at the amp/spkr
    public static final double SPK_LEFT_RPM = 5000;
    public static final double SPK_RIGHT_RPM = 3000;

    public static final double LEFT_SHUTTLING_RPM = 3700;
    public static final double RIGHT_SHUTTLING_RPM = 3500;

    // TODO: change these
    public static final double AMP_RIGHT_SPEED = -1200;
    public static final double AMP_LEFT_SPEED = -1200;

    public static final double SHOOTER_MAX_RPM = 6784;

    // TODO: tune further: get both faster
    public static final double SHOOTER_LEFT_kP = 0.0004;
    public static final double SHOOTER_LEFT_kI = 0.0;
    public static final double SHOOTER_LEFT_kD = 0.006;
    public static final double SHOOTER_LEFT_kFF = 0.000176;

    public static final double SHOOTER_RIGHT_kP = 0.0004;
    public static final double SHOOTER_RIGHT_kI = 0.0;
    public static final double SHOOTER_RIGHT_kD = 0.006; // increase
    public static final double SHOOTER_RIGHT_kFF = 0.000176;

    public static final int SHOOTER_STALL_LIMIT_CURRENT = 50; // in amps
    public static final int SHOOTER_FREE_SPEED_LIMIT = 5500; // in RPM

    public static final double SHOOTER_OUTTAKE_RPM = 1500;
  }

  public static class LimelightConstants {
    public static enum PIPELINE {
      APRILTAG_3D(0),
      NOTE(1),
      APRILTAG_2D(2);

      public int value;

      private PIPELINE(int value) {
        this.value = value;
      }
    }

    public static final String LIMELIGHT_NAME = "limelight";
    public static final double LIMELIGHT_TX_RANGE_DEG = 29.8; // range is thus -29.8 to 29.8
    public static final double LIMELIGHT_TY_RANGE_DEG = 24.85; // range is thus -24.85 to 24.85

    public static final double camHeightMeters = Units.inchesToMeters(78); // dont change
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

    public static final ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");
  }

  public static class IndexConstants {
    public static final int INDEX_MOTOR_ID = 4;
    public static final double INDEX_RUN_PCT = .35;
    public static final double INDEX_AMP_PCT = .23;

    public static boolean IS_TRAPPING = false;
  }

  public static DriverStation.Alliance allianceColor =
      Alliance.Blue; // blue by default, set in Robot.java
  public static int allianceLocation = 1; // 1 by default, set in Robot.java

  public static ShuffleboardTab diagnosticsTab = Shuffleboard.getTab("Diagnostics");
  public static ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

  public static Translation2d blueSpeakerPos = new Translation2d(0.076, 5.547868);
  public static Translation2d redSpeakerPos = new Translation2d(16.465042, 5.547868);
}
