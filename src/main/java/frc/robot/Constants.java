package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
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
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains =
        new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains =
        new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

    // pathplanner pids
    public static final PIDConstants AUTO_TRANS_PID = new PIDConstants(11.8, 0.8, 0.05);
    public static final PIDConstants AUTO_ROT_PID = new PIDConstants(8, 0, 0.15);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 6.21;

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
            .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 10;
    private static final int kFrontLeftSteerMotorId = 11;
    private static final int kFrontLeftEncoderId = 12;
    private static final double kFrontLeftEncoderOffset = 0.380859375;

    private static final double kFrontLeftXPosInches = 9.75;
    private static final double kFrontLeftYPosInches = 9.75;

    // Front Right
    private static final int kFrontRightDriveMotorId = 13;
    private static final int kFrontRightSteerMotorId = 14;
    private static final int kFrontRightEncoderId = 15;
    private static final double kFrontRightEncoderOffset = 0.474853515625;

    private static final double kFrontRightXPosInches = 9.75;
    private static final double kFrontRightYPosInches = -9.75;

    // Back Left
    private static final int kBackLeftDriveMotorId = 16;
    private static final int kBackLeftSteerMotorId = 2;
    private static final int kBackLeftEncoderId = 18;
    private static final double kBackLeftEncoderOffset = 0.447021484375;

    private static final double kBackLeftXPosInches = -9.75;
    private static final double kBackLeftYPosInches = 9.75;

    // Back Right
    private static final int kBackRightDriveMotorId = 19;
    private static final int kBackRightSteerMotorId = 20;
    private static final int kBackRightEncoderId = 21;
    private static final double kBackRightEncoderOffset = 0.366455078125;

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

  public static final int INTAKE_MOTOR_ID = 6; // TODO: change this id
  public static final int TOP_SHOOTER_MOTOR_ID = 2; // TODO: change this id
  public static final int BOTTOM_SHOOTER_MOTOR_ID = 3; // TODO: change this id
  public static final int SHOOTER_PIVOT_MOTOR_ID = -1; // TODO: change this id

  public static final double PIVOT_GEAR_RATIO = (1 / 4) * (1 / 5) * (9 / 66);

  public static final double PIVOT_kP = 2;
  public static final double PIVOT_kD = 0.3;
  public static final double PIVOT_kI = 0;

  public static final double PIVOT_START_POS = Units.degreesToRotations(30);
  public static final double PIVOT_CLIMB_UP_POS = Units.degreesToRotations(120);
  public static final double PIVOT_CLIMB_DOWN_POS = Units.degreesToRotations(40);

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

  public static DriverStation.Alliance allianceColor = null; // null by default, set in Robot.java
  public static int allianceLocation = -1; // -1 by default, set in Robot.java\

  public static ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

  public static double shooterSpeed = 28.06308713961776; // in ft/s
}
