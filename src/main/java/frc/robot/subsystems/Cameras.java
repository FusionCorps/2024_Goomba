package frc.robot.subsystems;

import static frc.robot.Constants.LimelightConstants.BLUE_SPK_TAG_ID;
import static frc.robot.Constants.LimelightConstants.RED_SPK_TAG_ID;
import static frc.robot.Constants.diagnosticsTab;
import static frc.robot.Constants.driverTab;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants.PIPELINE;
import frc.robot.util.UtilFunctions;
import java.util.Map;

public class Cameras extends SubsystemBase {
  public record BotPose(Pose3d pose, double latency, double tagCount, double avgTagDist) {}

  private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  // cached data
  private double tx = 0.0;
  private double ty = 0.0;
  private boolean hasTarget = false;
  private int fidID = 0;
  private Pose3d aprilTagTargetPose = new Pose3d();

  private BotPose botposeBlue = new BotPose(new Pose3d(), 0, 0, 0);
  private int pipelineNum = 0;

  // apriltag pose relative to robot space
  // horizontal, vertical, forward distances

  public Cameras() {
    limelightTable.getEntry("ledMode").setNumber(1); // turn off limelight LEDs
    limelightTable.getEntry("camMode").setNumber(0); // set limelight to vision processing mode
    limelightTable.getEntry("pipeline").setNumber(pipelineNum); // set limelight to default pipeline
    limelightTable
        .getEntry("camerapose_robotspace_set")
        .setDoubleArray(
            new double[] {
              -Units.inchesToMeters(16), // forward
              0.0, // right
              Units.inchesToMeters(18), // up
              180,
              32.39,
              0.0 // roll, pitch, yaw
            });

    setPriorityID(
        UtilFunctions.getAllianceColor() == Alliance.Blue ? BLUE_SPK_TAG_ID : RED_SPK_TAG_ID);

    driverTab
        .add("LL", new HttpCamera("limelight", "http://10.66.72.11:5801/stream.mjpg"))
        .withSize(4, 4)
        .withPosition(0, 0)
        .withProperties(Map.of("Show Crosshair", false, "Show Controls", false));

    diagnosticsTab.addDouble("tx", this::getTX);
    diagnosticsTab.addDouble("ty", this::getTY);
    diagnosticsTab.addBoolean("hasTarget", this::hasTarget);
    diagnosticsTab.addDouble("TZ 3D AprilTag", () -> this.aprilTagTargetPose.getZ());
    diagnosticsTab.addInteger("Pipeline #", this::getPipeline);
  }

  @Override
  public void periodic() {
    hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1;
    if (hasTarget) {
      tx = limelightTable.getEntry("tx").getDouble(0);
      ty = limelightTable.getEntry("ty").getDouble(0);
      fidID = (int) limelightTable.getEntry("id").getDouble(0);
      pipelineNum = (int) limelightTable.getEntry("getpipe").getDouble(0);
      // update apriltag pose and distances to apriltag
      if (pipelineNum == PIPELINE.APRILTAG_3D.value) {
        updatePrimaryAprilTagPose();
        // TODO: test this
        // updateBotPoseBlue();
      }
    }
  }

  // TODO: use megatag 2, empirically figure out std devs
  private void updateBotPoseBlue() {
    double[] data = limelightTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[11]);
    if (data.length < 11) {
      System.err.println("couldn't get latest botpose results");
      botposeBlue = new BotPose(new Pose3d(), 0, 0, 0);
    } else {
      botposeBlue =
          new BotPose(
              new Pose3d(
                  new Translation3d(data[0], data[1], data[2]),
                  new Rotation3d(
                      Units.degreesToRadians(data[3]),
                      Units.degreesToRadians(data[4]),
                      Units.degreesToRadians(data[5]))),
              data[6],
              data[7],
              data[9]);
    }
  }

  public BotPose getBotPoseBlue() {
    return botposeBlue;
  }

  /**
   * Gets the tx value from the limelight. tx is the horizontal offset from the crosshair to the
   * target, in degrees. tx > 0 means the target is to the left of the crosshair.
   *
   * @return tx (degrees)
   */
  public double getTX() {
    return hasTarget ? tx : 0.0;
  }

  public double getID() {
    return (hasTarget
            && (pipelineNum == PIPELINE.APRILTAG_3D.value
                || pipelineNum == PIPELINE.APRILTAG_2D.value))
        ? fidID
        : 0;
  }

  /**
   * Returns the ty value from the limelight. ty is the vertical offset from the crosshair to the
   * target, in degrees. ty > 0 means the target is above the crosshair.
   *
   * @return ty (degrees)
   */
  public double getTY() {
    return hasTarget ? ty : 0.0;
  }

  /**
   * Returns the tv value from the limelight. tv is 1 if the limelight has a valid target, 0
   * otherwise.
   *
   * @return tv (0 or 1)
   */
  public boolean hasTarget() {
    return hasTarget;
  }

  /**
   * Sets the Limelight pipeline.
   *
   * @param pipeline integer (0-9)
   */
  public void setPipeline(int pipeline) {
    limelightTable.getEntry("pipeline").setNumber(pipeline);
  }

  public void setPriorityID(int id) {
    limelightTable.getEntry("priorityid").setNumber(id);
  }

  /**
   * Returns the current Limelight pipeline.
   *
   * @return pipeline integer (0-9)
   */
  public int getPipeline() {
    return pipelineNum;
  }

  /**
   * Returns the last known pose of the primary apriltag target
   *
   * @return
   */
  public Pose3d getPrimaryAprilTagPose() {
    return aprilTagTargetPose;
  }

  private void updatePrimaryAprilTagPose() {
    try {
      // get latest apriltag pose results, if on correct pipeline and target seen
      if (hasTarget() && getPipeline() == PIPELINE.APRILTAG_3D.value) {
        double[] dataPose =
            limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[0]);
        if (dataPose.length < 6) aprilTagTargetPose = new Pose3d();
        else {
          aprilTagTargetPose =
              new Pose3d(
                  new Translation3d(dataPose[0], dataPose[1], dataPose[2]),
                  new Rotation3d(
                      Units.degreesToRadians(dataPose[3]),
                      Units.degreesToRadians(dataPose[4]),
                      Units.degreesToRadians(dataPose[5])));
        }
      }
    } catch (Exception e) {
      System.err.println("couldn't get latest apritag pose results");
    }
  }
}
