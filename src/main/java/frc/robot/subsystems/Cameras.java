package frc.robot.subsystems;

import static frc.robot.Constants.limelightTab;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;


public class Cameras extends SubsystemBase {
    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    public double distToAprilTag = 0.0; // in meters
    private Pose3d aprilTagTargetPose = new Pose3d();

    private JSONParser parser = new JSONParser();
    private String jsonString;

    public Cameras() {
        limelightTable.getEntry("ledMode").setNumber(1); // turn off limelight LEDs
        limelightTable.getEntry("camMode").setNumber(0); // set limelight to vision processing mode
        limelightTable.getEntry("pipeline").setNumber(0); // set limelight to default pipeline
        limelightTable.getEntry("camerapose_robotspace_set").setDoubleArray(new double[] {
            0.12, // forward
            0.0, // right
            1.3716, // up
            0, 1, 0 // roll, pitch, yaw
        });

        limelightTab.addDouble("tx", this::getTX);
        limelightTab.addDouble("ty", this::getTY);
        limelightTab.addBoolean("hasTarget", this::hasTarget);
        limelightTab.addDouble("Distance to AprilTag", () -> distToAprilTag);
        limelightTab.addInteger("Pipeline", this::getPipeline);
    }

    @Override
    public void periodic() {
        try {
            getPrimaryAprilTagPose(); // update apriltag pose
            distToAprilTag = aprilTagTargetPose.getZ(); // update distToTarget
        } catch (Exception e) {
            System.err.println("couldn't get latest apritag pose results");
        }
    }

    /**
    * Gets the tx value from the limelight.
    * tx is the horizontal offset from the crosshair to the target, in degrees.
    * tx > 0 means the target is to the left of the crosshair.
    * @return tx (degrees)
    */
    public double getTX() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    /**
    * Returns the ty value from the limelight.
    * ty is the vertical offset from the crosshair to the target, in degrees.
    * ty > 0 means the target is above the crosshair.
    * @return ty (degrees)
    */
    public double getTY() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    /**
     * Returns the tv value from the limelight.
     * tv is 1 if the limelight has a valid target, 0 otherwise.
     * @return tv (0 or 1)
     */
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    /**
     * Sets the Limelight pipeline.
     * @param pipeline integer (0-9)
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Returns the current Limelight pipeline.
     * @return pipeline integer (0-9)
     */
    public int getPipeline() {
        return (int) limelightTable.getEntry("getpipe").getDouble(0);
    }

    public Pose3d getPrimaryAprilTagPose() {
        // get latest apriltag pose results, if on correct pipeline and target seen
        if (hasTarget() && getPipeline() == 0) {
            var fid = LimelightHelpers.getLatestResults(Constants.LIMELIGHT_NAME).targetingResults.targets_Fiducials;
            aprilTagTargetPose = fid[0].getTargetPose_RobotSpace();
            return aprilTagTargetPose;
        }
        else return null;

        // TODO: check if this alternative logic works
        // acc to limelight docs, works, but key doesn't appear in networktables???
            // double[] targetPoseArray = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[0]);
            // // this works the same way
            // targetPoseArray = LimelightHelpers.getLimelightNTDoubleArray(Constants.LIMELIGHT_NAME, "targetpose_robotspace");
            // targetPose = new Pose3d(
            //     targetPoseArray[0],
            //     targetPoseArray[1],
            //     targetPoseArray[2],
            //     new Rotation3d(
            //         targetPoseArray[3],
            //         targetPoseArray[4],
            //         targetPoseArray[5]
            //     )
            // );

            // TODO: check if this alternative logic works
            // manually parse json to get rid of delay??
            // jsonString = limelightTable.getEntry("json").getString("");
            // JSONObject json = (JSONObject) parser.parse(jsonString);
            // JSONArray targetPoseJSON = (JSONArray) ((JSONObject) ((JSONArray)(
            //     (JSONObject) json.get("Results"))
            //         .get("Fiducial")).get(0)).get("t6t_rs");
            
            // targetPose = new Pose3d(
            //     (double) targetPoseJSON.get(0),
            //     (double) targetPoseJSON.get(1),
            //     (double) targetPoseJSON.get(2),
            //     new Rotation3d(
            //         (double) targetPoseJSON.get(3),
            //         (double) targetPoseJSON.get(4),
            //         (double) targetPoseJSON.get(5)
            //     )
            // );
    }
}
