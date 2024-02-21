package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.MaxSpeed;
import static frc.robot.Constants.LimelightConstants.LIMELIGHT_NAME;
import static frc.robot.Constants.diagnosticsTab;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SysIdSwerveRotation;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SysIdSwerveSteerGains;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SysIdSwerveTranslation;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
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
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LimelightConstants.PIPELINE;
import frc.robot.LimelightHelpers;
import frc.robot.commands.swerve.vision.RotateToAngle;

/**
 * This class represents a command-based swerve drivetrain subsystem. It extends the
 * SwerveDrivetrain class and implements the Subsystem interface. It provides methods for
 * controlling the swerve drivetrain, updating odometry, and executing various commands related to
 * the drivetrain.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
  public Cameras mCamera;

  SysIdSwerveTranslation sysIdTransReq = new SysIdSwerveTranslation();
  SysIdSwerveSteerGains sysIdSteerReq = new SysIdSwerveSteerGains();
  SysIdSwerveRotation sysIdRotReq = new SysIdSwerveRotation();

  SysIdRoutine sysIdTransRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Units.Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> this.setControl(sysIdTransReq.withVolts(volts)), null, this));

  SysIdRoutine sysIdSteerRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Units.Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> this.setControl(sysIdSteerReq.withVolts(volts)), null, this));

  SysIdRoutine sysIdRotRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Units.Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> this.setControl(sysIdRotReq.withVolts(volts)), null, this));

  // simulation variables
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  // telemetry for the swerve drivetrain
  private Field2d field2d = new Field2d();
  // use networktables for things that cannot be displayed on shuffleboard
  private NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Drivetrain");
  private StructArrayPublisher<SwerveModuleState> moduleStates =
      driveTable.getStructArrayTopic("Module States", SwerveModuleState.struct).publish();
  private StructArrayPublisher<SwerveModuleState> desiredStates =
      driveTable.getStructArrayTopic("Desired States", SwerveModuleState.struct).publish();
  private StructPublisher<Pose3d> pose3D =
      driveTable.getStructTopic("3D Pose", Pose3d.struct).publish();

  /* Mechanisms to represent the swerve module states */
  private Mechanism2d[] m_moduleMechanisms =
      new Mechanism2d[] {
        new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1),
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
  private DoubleEntry robotVelocityX = driveTable.getDoubleTopic("Robot Velocity X").getEntry(0.0);
  private DoubleEntry robotVelocityY = driveTable.getDoubleTopic("Robot Velocity Y").getEntry(0.0);

  public Drivetrain(
      Cameras camera,
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    mCamera = camera;
    configPathPlanner();

    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          field2d.getObject("autoTrajectory").setPoses(poses);
        });

    // vision measurement std filter
    // setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    if (Utils.isSimulation()) startSimThread();

    // add diagnostics telemetry to shuffleboard
    diagnosticsTab.add(field2d);

    diagnosticsTab.addDouble(
        "Odometry Frequency", () -> truncPlaces(odometryFrequency.get(0.0), 2));
    diagnosticsTab.addDouble("Robot Speed", () -> truncPlaces(robotSpeed.get(0.0), 2));
    diagnosticsTab.addDouble("Robot Velocity X", () -> truncPlaces(robotVelocityX.get(0.0), 2));
    diagnosticsTab.addDouble("Robot Velocity Y", () -> truncPlaces(robotVelocityY.get(0.0), 2));

    registerTelemetry(this::telemeterize);
  }

  @Override
  public void periodic() {
    // updates the odometry from aprilTag data
    updateOdometryFromAprilTags(1.0);
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
      Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(LIMELIGHT_NAME);
      // System.out.println("Pose update: " + pose);
      // double timestamp = Timer.getFPGATimestamp();
      double timestamp =
          Timer.getFPGATimestamp()
              - LimelightHelpers.getLatency_Capture(LIMELIGHT_NAME) / 1000.0
              - LimelightHelpers.getLatency_Pipeline(LIMELIGHT_NAME) / 1000.0;
      addVisionMeasurement(pose, timestamp);
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
            DrivetrainConstants.AUTO_TRANS_PID,
            DrivetrainConstants.AUTO_ROT_PID,
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
    pose3D.set(new Pose3d(getPose()));
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
              .schedule(
                  new RotateToAngle(
                      this,
                      angle,
                      Constants.StageAlignment.toleranceDeg,
                      Constants.StageAlignment.runTime));
        });
  }

  private SysIdRoutine sysIdRoutineToApply = sysIdTransRoutine;

  public Command sysIDDynamic(Direction direction) {
    return sysIdRoutineToApply.dynamic(direction);
  }

  public Command sysIDQuasistatic(Direction direction) {
    return sysIdRoutineToApply.quasistatic(direction);
  }
}
