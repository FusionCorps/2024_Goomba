package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.MaxSpeed;
import static frc.robot.Constants.LimelightConstants.MAX_TAG_DIST_ODO;
import static frc.robot.Constants.LimelightConstants.MULTI_TAG_XY_STD_DEV;
import static frc.robot.Constants.LimelightConstants.SINGLE_TAG_DIST_ODO;
import static frc.robot.Constants.LimelightConstants.SINGLE_TAG_XY_STD_DEV;
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
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
import frc.robot.commands.swerve.manual.RunSwerveFC;
import frc.robot.commands.swerve.manual.RunSwerveRC;
import frc.robot.commands.swerve.vision.RotateToAngle;
import frc.robot.subsystems.Cameras.BotPose;
import frc.robot.util.UtilFunctions;

/**
 * This class represents a command-based swerve drivetrain subsystem. It extends the
 * SwerveDrivetrain class and implements the Subsystem interface. It provides methods for
 * controlling the swerve drivetrain, updating odometry, and executing various commands related to
 * the drivetrain.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
  public Cameras mCamera;

  // simulation variables
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  public DriveTelemetry driveTelemetry = new DriveTelemetry(this);

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
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimit(70)
                  .withSupplyCurrentLimit(60)
                  .withSupplyCurrentThreshold(80)
                  .withSupplyTimeThreshold(0.5));
    }

    getDaqThread().setThreadPriority(99);
    mCamera = camera;
    configPathPlanner();

    if (Utils.isSimulation()) startSimThread();

    registerTelemetry(driveTelemetry::telemeterize);
  }

  @Override
  public void periodic() {
    // LimelightHelpers.SetRobotOrientation(LIMELIGHT_NAME, getPose().getRotation().getDegrees(), 0,
    // 0, 0, 0, 0);

    // If the robot is disabled or the operator perspective has not been applied
    // yet, apply correct
    // heading for field-centric driving
    // if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
    // this.setOperatorPerspectiveForward(
    // UtilFunctions.getAllianceColor() == Alliance.Red
    // ? RedAlliancePerspectiveRotation
    // : BlueAlliancePerspectiveRotation);
    // hasAppliedOperatorPerspective = true;
    // }

    // TODO: updates the odometry from aprilTag data
    // updateOdometryFromAprilTags();
  }

  /**
   * Updates the odometry from the latest april tag data. If the robot is real and the limelight
   * detects a target, the robot's position and orientation will be updated.
   *
   * @param maxDist the maximum distance to the target in meters required for odometry to be
   *     updated, recommended <= 1.0
   */
  private void updateOdometryFromAprilTags() {
    if (RobotBase.isReal()
        && mCamera.hasTarget()
        && mCamera.getPipeline() == PIPELINE.APRILTAG_3D.value) {
      BotPose llBotposeData = mCamera.getBotPoseBlue();

      double xyStdDevs = 0.0;
      double headingStdDevs = 999999; // mostly trust gyro over ll heading

      if (llBotposeData.avgTagDist() > MAX_TAG_DIST_ODO) return; // generally too far
      if (llBotposeData.tagCount() >= 2) // multi-tag, in OK range
      xyStdDevs = MULTI_TAG_XY_STD_DEV;
      else if (llBotposeData.avgTagDist() >= SINGLE_TAG_DIST_ODO) return; // single-tag, too far
      else xyStdDevs = SINGLE_TAG_XY_STD_DEV; // single-tag, in OK range

      addVisionMeasurement(
          llBotposeData.pose().toPose2d(),
          llBotposeData.latency(),
          VecBuilder.fill(xyStdDevs, xyStdDevs, headingStdDevs));
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
        () -> UtilFunctions.getAllianceColor() == Alliance.Red,
        this);
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

  private double truncPlaces(double value, int numOfPlaces) {
    double scale = Math.pow(10, numOfPlaces);
    return Math.round(value * scale) / scale;
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

  public class DriveTelemetry {
    // telemetry for the swerve drivetrain
    public Field2d field2d = new Field2d();
    // use networktables for things that cannot be displayed on shuffleboard
    private NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Drivetrain");
    private StructArrayPublisher<SwerveModuleState> moduleStates =
        driveTable.getStructArrayTopic("Module States", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> desiredStates =
        driveTable.getStructArrayTopic("Desired States", SwerveModuleState.struct).publish();
    private StructPublisher<Pose3d> pose3D =
        driveTable.getStructTopic("3D Pose", Pose3d.struct).publish();
    private StructPublisher<Pose2d> pose2D =
        driveTable.getStructTopic("2D Pose", Pose2d.struct).publish();

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
    private DoubleEntry odometryFrequency =
        driveTable.getDoubleTopic("Odometry Frequency").getEntry(0.0);
    private DoubleEntry robotSpeed = driveTable.getDoubleTopic("Robot Speed").getEntry(0.0);
    private DoubleEntry robotVelocityX =
        driveTable.getDoubleTopic("Robot Velocity X").getEntry(0.0);
    private DoubleEntry robotVelocityY =
        driveTable.getDoubleTopic("Robot Velocity Y").getEntry(0.0);

    public DriveTelemetry(Drivetrain drivetrain) {
      PathPlannerLogging.setLogActivePathCallback(
          (poses) -> {
            field2d.getObject("path").setPoses(poses);
          });
      PathPlannerLogging.setLogCurrentPoseCallback((pose) -> field2d.setRobotPose(pose));
      PathPlannerLogging.setLogTargetPoseCallback(
          (pose) -> field2d.getObject("target").setPose(pose));

      // add diagnostics telemetry to shuffleboard
      diagnosticsTab.add(field2d);
      SendableChooser<Command> driveMode = new SendableChooser<>();
      driveMode.setDefaultOption("Field-Centric", new RunSwerveFC(drivetrain));
      driveMode.addOption("Robot-Centric", new RunSwerveRC(drivetrain));
      driveMode.onChange((newDriveMode) -> drivetrain.setDefaultCommand(newDriveMode));
      diagnosticsTab.add("Drive Mode Chooser", driveMode);

      diagnosticsTab.addDouble(
          "Odometry Frequency", () -> truncPlaces(odometryFrequency.get(0.0), 2));
      diagnosticsTab.addDouble("Robot Speed", () -> truncPlaces(robotSpeed.get(0.0), 2));
      diagnosticsTab.addDouble("Robot Velocity X", () -> truncPlaces(robotVelocityX.get(0.0), 2));
      diagnosticsTab.addDouble("Robot Velocity Y", () -> truncPlaces(robotVelocityY.get(0.0), 2));
      diagnosticsTab.addDouble("Yaw", () -> m_yawGetter.getValueAsDouble());
    }

    private void telemeterize(SwerveDriveState state) {
      Pose2d pose = state.Pose;
      field2d.setRobotPose(pose);

      odometryFrequency.set(1.0 / state.OdometryPeriod);
      robotSpeed.set(Math.hypot(state.speeds.vxMetersPerSecond, state.speeds.vyMetersPerSecond));
      robotVelocityX.set(state.speeds.vxMetersPerSecond);
      robotVelocityY.set(state.speeds.vyMetersPerSecond);

      /* Telemeterize the module's states */
      for (int i = 0; i < 4; i++) {
        m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
        m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
        m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
      }

      moduleStates.set(state.ModuleStates);
      desiredStates.set(state.ModuleTargets);
      pose3D.set(new Pose3d(pose));
      pose2D.set(pose);
    }
  }
}
