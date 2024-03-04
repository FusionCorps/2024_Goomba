package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.MaxSpeed;
import static frc.robot.Constants.LimelightConstants.LIMELIGHT_NAME;
import static frc.robot.Constants.diagnosticsTab;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LimelightConstants.PIPELINE;
import frc.robot.LimelightHelpers;
import frc.robot.commands.swerve.manual.RunSwerveFC;
import frc.robot.commands.swerve.manual.RunSwerveRC;
import frc.robot.commands.swerve.vision.RotateToAngle;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * This class represents a command-based swerve drivetrain subsystem. It extends the
 * SwerveDrivetrain class and implements the Subsystem interface. It provides methods for
 * controlling the swerve drivetrain, updating odometry, and executing various commands related to
 * the drivetrain.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
  @AutoLog
  public static class DrivetrainInputs {
    public double odometryFrequency = 0.0;
    public double robotSpeed = 0.0;
    public double robotVelocityX = 0.0;
    public double robotVelocityY = 0.0;
    public double yaw = 0.0;
  }

  public DrivetrainInputsAutoLogged inputs = new DrivetrainInputsAutoLogged();

  public Cameras mCamera;

  // simulation variables
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  DriveTelemetry driveTelemetry = new DriveTelemetry(this);

  public Drivetrain(
      Cameras camera,
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

    for (int i = 0; i < 4; i++) {
      SwerveModule module = getModule(i);
      module
          .getDriveMotor()
          .getConfigurator()
          .apply(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentThreshold(80)
                  .withSupplyTimeThreshold(0.75)); // per 2023-Ignition code, TODO: test this
    }

    getDaqThread().setThreadPriority(99);
    mCamera = camera;
    configPathPlanner();

    // vision measurement std filter, per LL docs (fully trusts internal rotation measurements)
    setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));

    if (Utils.isSimulation()) startSimThread();

    registerTelemetry(driveTelemetry::telemeterize);
  }

  @Override
  public void periodic() {
    // updates the odometry from aprilTag data
    updateOdometryFromAprilTags(1.0);

    inputs.odometryFrequency = 1.0 / getState().OdometryPeriod;
    inputs.robotSpeed =
        Math.hypot(getState().speeds.vxMetersPerSecond, getState().speeds.vyMetersPerSecond);
    inputs.robotVelocityX = getState().speeds.vxMetersPerSecond;
    inputs.robotVelocityY = getState().speeds.vyMetersPerSecond;
    inputs.yaw = getState().Pose.getRotation().getDegrees();
  }

  /**
   * Updates the odometry from the latest april tag data. If the robot is real and the limelight
   * detects a target, the robot's position and orientation will be updated.
   *
   * @param maxDist the maximum distance to the target in meters required for odometry to be
   *     updated, recommended <= 1.0
   */
  private void updateOdometryFromAprilTags(double maxDist) {
    if (RobotBase.isReal()
        && mCamera.hasTarget()
        && mCamera.getPipeline() == PIPELINE.APRILTAG_3D.value
        && mCamera.getPrimaryAprilTagPose().getZ() < maxDist) {

      LimelightHelpers.PoseEstimate limelightMeasurement =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

      addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
    }
  }

  /** Configures the PathPlanner's AutoBuilder. */
  private void configPathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::seedFieldRelative,
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        new HolonomicPathFollowerConfig(
            DrivetrainConstants.AUTO_DRIVE_PID,
            DrivetrainConstants.AUTO_STEER_PID,
            DrivetrainConstants.MaxSpeed,
            DrivetrainConstants.DRIVEBASE_RADIUS,
            new ReplanningConfig()),
        this::isAllianceRed,
        this);
  }

  /**
   * @return true if the robot is on the red alliance, false if on the blue alliance or if the
   *     alliance color is unknown
   */
  private boolean isAllianceRed() {
    // Boolean supplier that controls when the path will be mirrored for the red alliance
    // This will flip the path being followed to the red side of the field.
    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    if (Constants.allianceColor != null) {
      return Constants.allianceColor == DriverStation.Alliance.Red;
    }
    return false;
  }

  /**
   * Drives the robot relative to its current orientation using the specified speeds.
   *
   * @param speeds The desired chassis speeds in meters per second.
   */
  private void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveRequest request =
        new SwerveRequest.RobotCentric()
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond);
    setControl(request);
  }

  /**
   * @return The current pose of the robot.
   */
  @AutoLogOutput
  public Pose2d getPose() {
    return getState().Pose;
  }

  /**
   * @return The current robot-relative chassis speeds of the robot.
   */
  private ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  /** Zeroes the gyro yaw. */
  public void resetGyro() {
    m_pigeon2.reset();
  }

  /**
   * @return the drivetrain camera
   */
  public Cameras getCamera() {
    return mCamera;
  }

  public Pose2d getVelocity() {
    return new Pose2d(
        getState().speeds.vxMetersPerSecond,
        getState().speeds.vyMetersPerSecond,
        Rotation2d.fromRotations(getState().speeds.omegaRadiansPerSecond));
  }

  private void startSimThread() {
    System.out.println("Starting sim thread");
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  // method that rotates the robot a certain angle to face the stage
  public Command rotateInStageCommand() {
    return new InstantCommand(
        () -> {
          double angle = 60;

          int id = (int) mCamera.getID();

          if (id == 14 || id == 13) {
            angle = 180;
          } else if (id == 15 || id == 12) {
            angle = -60;
          }
          CommandScheduler.getInstance()
              .schedule(new RotateToAngle(this, angle, Constants.StageAlignment.toleranceDeg));
        });
  }

  private class DriveTelemetry {
    /* Mechanisms to represent the swerve module states */
    private Mechanism2d[] m_moduleMechanisms =
        new Mechanism2d[] {
          new Mechanism2d(1, 1),
          new Mechanism2d(1, 1),
          new Mechanism2d(1, 1),
          new Mechanism2d(1, 1),
        };
    /* A direction and length changing ligament for speed representation */
    private MechanismLigament2d[] m_moduleSpeeds =
        new MechanismLigament2d[] {
          m_moduleMechanisms[0]
              .getRoot("RootSpeed", 0.5, 0.5)
              .append(new MechanismLigament2d("Speed", 0.5, 0)),
          m_moduleMechanisms[1]
              .getRoot("RootSpeed", 0.5, 0.5)
              .append(new MechanismLigament2d("Speed", 0.5, 0)),
          m_moduleMechanisms[2]
              .getRoot("RootSpeed", 0.5, 0.5)
              .append(new MechanismLigament2d("Speed", 0.5, 0)),
          m_moduleMechanisms[3]
              .getRoot("RootSpeed", 0.5, 0.5)
              .append(new MechanismLigament2d("Speed", 0.5, 0)),
        };
    /* A direction changing and length constant ligament for module direction */
    private MechanismLigament2d[] m_moduleDirections =
        new MechanismLigament2d[] {
          m_moduleMechanisms[0]
              .getRoot("RootDirection", 0.5, 0.5)
              .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
          m_moduleMechanisms[1]
              .getRoot("RootDirection", 0.5, 0.5)
              .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
          m_moduleMechanisms[2]
              .getRoot("RootDirection", 0.5, 0.5)
              .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
          m_moduleMechanisms[3]
              .getRoot("RootDirection", 0.5, 0.5)
              .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        };

    public DriveTelemetry(Drivetrain drivetrain) {
      SendableChooser<Command> driveMode = new SendableChooser<>();
      driveMode.setDefaultOption("Field-Centric", new RunSwerveFC(drivetrain));
      driveMode.addOption("Robot-Centric", new RunSwerveRC(drivetrain));
      driveMode.onChange((newDriveMode) -> drivetrain.setDefaultCommand(newDriveMode));
      diagnosticsTab.add("Drive Mode Chooser", driveMode);
    }

    private void telemeterize(SwerveDriveState state) {
      /* Telemeterize the module's states */
      for (int i = 0; i < 4; i++) {
        m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
        m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
        m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
      }

      Logger.recordOutput("Module States", state.ModuleStates);
      Logger.recordOutput("Desired Module States", state.ModuleTargets);
      Logger.recordOutput("Robot Pose2d", state.Pose);
      Logger.recordOutput("Robot Pose3d", new Pose3d(getPose()));
      Logger.recordOutput("FL Swerve", m_moduleMechanisms[0]);
      Logger.recordOutput("FR Swerve", m_moduleMechanisms[1]);
      Logger.recordOutput("BL Swerve", m_moduleMechanisms[2]);
      Logger.recordOutput("BR Swerve", m_moduleMechanisms[3]);
    }
  }
}
