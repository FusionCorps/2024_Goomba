package frc.robot.commands.pivot;

import static frc.robot.Constants.PivotConstants.PIVOT_ANGLES_MAP;
import static frc.robot.Constants.diagnosticsTab;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Pivot;

/**
 * Aims the pivot using a manually tuned look-up table, relating depth distance
 * to speaker apriltag
 * with pivot angles for shooting.
 */
public class AutoPivotAim extends Command {
  Pivot mPivot;
  Cameras mCamera;
  Index mIndex;
  double errorThreshold = 0.2;
  double distanceToAprilTag = 0.0;
  double angleToSet = 0.0;
/**
 * @param pivot
 * @param camera
 * @param index
 * @param defaultAngle
 */
  public AutoPivotAim(Pivot pivot, Cameras camera, Index index, double defaultAngle) {
    mCamera = camera;
    mPivot = pivot;
    mIndex = index;
    angleToSet = defaultAngle;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    // diagnosticsTab.addDouble("AutoPivotAim angle", () -> angleToSet); // TODO:
    // might crash
  }

  @Override
  public void execute() {
    if (mCamera.hasTarget()) {
      distanceToAprilTag = mCamera.getPrimaryAprilTagPose().getZ();
      if (distanceToAprilTag != 0.0)
        angleToSet = PIVOT_ANGLES_MAP.get(distanceToAprilTag);
    }
    mPivot.setPivotAngle(angleToSet);
  }

  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomous()) {
      return Math.abs(mPivot.getPivotAngle() - PIVOT_ANGLES_MAP.get(distanceToAprilTag)) < errorThreshold;
    } else {
      return !mIndex.beamBroken();
    }
  }
}
