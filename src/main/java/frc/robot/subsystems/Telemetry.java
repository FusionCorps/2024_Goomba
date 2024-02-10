package frc.robot.subsystems;

import static frc.robot.Constants.LimelightConstants.limelightTab;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Map;

public class Telemetry {
  private final double MaxSpeed;

  private Field2d field2d = new Field2d(); // for visualizing robot position and trajectories
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");

  private StructArrayPublisher<SwerveModuleState> moduleStates =
      table.getStructArrayTopic("Module States", SwerveModuleState.struct).publish();
  private StructArrayPublisher<SwerveModuleState> desiredStates =
      table.getStructArrayTopic("Desired States", SwerveModuleState.struct).publish();

  /* Keep a reference of the last pose to calculate the speeds */
  private Pose2d m_lastPose = new Pose2d();
  private double lastTime = Utils.getCurrentTimeSeconds();

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

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   *
   * @param maxSpeed Maximum speed in meters per second
   */
  public Telemetry(double maxSpeed) {
    MaxSpeed = maxSpeed;
    SmartDashboard.putData(field2d);

    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          field2d.getObject("autoTrajectory").setPoses(poses);
        });

    for (int i = 0; i < 4; i++) {
      Shuffleboard.getTab("Module Mechanisms").add("Module " + i, m_moduleMechanisms[i]);
    }

    HttpCamera limelightFeed =
        new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
    limelightTab
        .add("LL", limelightFeed)
        .withPosition(0, 0)
        .withSize(15, 8)
        .withProperties(Map.of("Show Crosshair", true, "Show Controls", true));
  }

  /* Accept the swerve drive state and telemeterize it to smartdashboard */
  /**
   * Telemeterize the swerve drive state to smartdashboard
   *
   * @param state
   */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the pose */
    Pose2d pose = state.Pose;
    field2d.setRobotPose(pose);

    /* Telemeterize the robot's general speeds */
    double currentTime = Utils.getCurrentTimeSeconds();
    double diffTime = currentTime - lastTime;
    lastTime = currentTime;
    Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
    m_lastPose = pose;

    Translation2d velocities =
        distanceDiff.div(diffTime); // divide displacement by time to get velocity

    /* Telemeterize the module's states */
    for (int i = 0; i < 4; i++) {
      m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
    }

    SmartDashboard.putNumber("Odometry Frequency", 1.0 / state.OdometryPeriod);
    SmartDashboard.putNumber("Robot Speed", velocities.getNorm());
    SmartDashboard.putNumber("Robot Velocity X", velocities.getX());
    SmartDashboard.putNumber("Robot Velocity Y", velocities.getY());

    moduleStates.set(state.ModuleStates);
    desiredStates.set(state.ModuleTargets);
  }
}
