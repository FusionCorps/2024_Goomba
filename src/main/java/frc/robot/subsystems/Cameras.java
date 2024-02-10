package frc.robot.subsystems;

import static frc.robot.Constants.LimelightConstants.LIMELIGHT_NAME;
import static frc.robot.Constants.LimelightConstants.limelightTab;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Cameras extends SubsystemBase {
  private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private double distToAprilTagHorizontal = 0.0; // in meters
  private double distToAprilTagVertical = 0.0; // in meters

  private Pose3d aprilTagTargetPose = new Pose3d();

  public Cameras() {
    limelightTable.getEntry("ledMode").setNumber(1); // turn off limelight LEDs
    limelightTable.getEntry("camMode").setNumber(0); // set limelight to vision processing mode
    limelightTable.getEntry("pipeline").setNumber(0); // set limelight to default pipeline
    limelightTable
        .getEntry("camerapose_robotspace_set")
        .setDoubleArray(
            new double[] {
              0.12, // forward
              0.0, // right
              1.3716, // up
              0,
              1,
              0 // roll, pitch, yaw
            });

    limelightTab.addDouble("tx", this::getTX);
    limelightTab.addDouble("ty", this::getTY);
    limelightTab.addBoolean("hasTarget", this::hasTarget);
    limelightTab.addDouble("Horizontal Distance to AprilTag", () -> distToAprilTagHorizontal);
    limelightTab.addInteger("Pipeline", this::getPipeline);
  }

  @Override
  public void periodic() {
    // update apriltag pose and distances to apriltag
    try {
      getPrimaryAprilTagPose();
      distToAprilTagHorizontal = aprilTagTargetPose.getZ();
      distToAprilTagVertical = aprilTagTargetPose.getY();
      // System.out.println(distToAprilTag);
    } catch (Exception e) {
      System.err.println("couldn't get latest apritag pose results");
    }
  }

  public double getDistToAprilTagHorizontal() {
    return distToAprilTagHorizontal;
  }

  public double getDistToAprilTagVertical() {
    return distToAprilTagVertical;
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
      return aprilTagTargetPose;
    } else return null;
  }
}
