package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Cameras extends SubsystemBase {

    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    double distToTargetMeters = 0.0;

    public Cameras(){
        
    }

    @Override
    public void periodic(){
        if (hasTarget()) {
            distToTargetMeters = LimelightHelpers.getTargetPose_RobotSpace(Constants.LIMELIGHT_NAME)[2];
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
