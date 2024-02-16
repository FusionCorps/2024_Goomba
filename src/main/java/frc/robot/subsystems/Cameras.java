package frc.robot.subsystems;

import static frc.robot.Constants.LimelightConstants.LIMELIGHT_NAME;
import static frc.robot.Constants.diagnosticsTab;
import static frc.robot.Constants.driverTab;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import java.util.Map;

public class Cameras extends SubsystemBase {
  private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  // apriltag pose relative to robot space
  // horizontal, vertical, forward distances
  private Pose3d aprilTagTargetPose = new Pose3d();

  public Cameras() {
    limelightTable.getEntry("ledMode").setNumber(1); // turn off limelight LEDs
    limelightTable.getEntry("camMode").setNumber(0); // set limelight to vision processing mode
    limelightTable.getEntry("pipeline").setNumber(0); // set limelight to default pipeline
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

    driverTab
        .add("LL", new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg"))
        .withSize(4, 4)
        .withPosition(0, 0)
        .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

    diagnosticsTab.addDouble("tx", this::getTX);
    diagnosticsTab.addDouble("ty", this::getTY);
    diagnosticsTab.addBoolean("hasTarget", this::hasTarget);
    diagnosticsTab.addDouble("TZ 3D AprilTag", () -> this.aprilTagTargetPose.getX());
    diagnosticsTab.addInteger("Pipeline #", this::getPipeline);
  }

  @Override
  public void periodic() {
    // update apriltag pose and distances to apriltag
    // try {
    //   getPrimaryAprilTagPose();
    //   // System.out.println(distToAprilTag);
    // } catch (Exception e) {
    //   System.err.println("couldn't get latest apritag pose results");
    // }
  }

  /**
   * Gets the tx value from the limelight. tx is the horizontal offset from the crosshair to the
   * target, in degrees. tx > 0 means the target is to the left of the crosshair.
   *
   * @return tx (degrees)
   */
  public double getTX() {
    if (hasTarget()) {
      return limelightTable.getEntry("tx").getDouble(0.0);
    } else {
      return 0.0;
    }
  }

  public double getID() {
    return LimelightHelpers.getFiducialID(LIMELIGHT_NAME);
  }

  /**
   * Returns the ty value from the limelight. ty is the vertical offset from the crosshair to the
   * target, in degrees. ty > 0 means the target is above the crosshair.
   *
   * @return ty (degrees)
   */
  public double getTY() {
    return limelightTable.getEntry("ty").getDouble(0.0);
  }

  /**
   * Returns the tv value from the limelight. tv is 1 if the limelight has a valid target, 0
   * otherwise.
   *
   * @return tv (0 or 1)
   */
  public boolean hasTarget() {
    return limelightTable.getEntry("tv").getDouble(0) == 1;
  }

  /**
   * Sets the Limelight pipeline.
   *
   * @param pipeline integer (0-9)
   */
  public void setPipeline(int pipeline) {
    limelightTable.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Returns the current Limelight pipeline.
   *
   * @return pipeline integer (0-9)
   */
  public int getPipeline() {
    return (int) limelightTable.getEntry("getpipe").getDouble(0);
  }

  public Pose3d getPrimaryAprilTagPose() {
    // get latest apriltag pose results, if on correct pipeline and target seen
    if (hasTarget() && getPipeline() == 0) {
      var fid =
          LimelightHelpers.getLatestResults(LIMELIGHT_NAME).targetingResults.targets_Fiducials;
      aprilTagTargetPose = fid[0].getTargetPose_RobotSpace();
    }
    return aprilTagTargetPose; // will return either updated or old pose
  }
}
