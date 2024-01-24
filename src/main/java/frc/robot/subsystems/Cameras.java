package frc.robot.subsystems;

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
    
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    double distToTargetMeters = 0.0;
    Pose3d targetPose = new Pose3d();

    JSONParser parser = new JSONParser();
    String jsonString;

    public Cameras() {
        
    }

    @Override
    public void periodic() {
        if (hasTarget()) {
            try {
                var fid = LimelightHelpers.getLatestResults(Constants.LIMELIGHT_NAME).targetingResults.targets_Fiducials;
                targetPose = fid[0].getTargetPose_RobotSpace();

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

            } catch (Exception e) {
                System.out.println("couldn't get latest target results");
            }
            distToTargetMeters = targetPose.getZ();
        }


        SmartDashboard.putNumber("distanceToTarget", distToTargetMeters);
    }

    public double getDistanceToAprilTag() {
        if (hasTarget())
            return distToTargetMeters;
        else return 0.0;
    }

    /**
    * Gets the tx value from the limelight.
    * tx is the horizontal offset from the crosshair to the target, in degrees.
    * tx > 0 means the target is to the right of the crosshair.
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


    
}
