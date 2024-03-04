package frc.robot.commands.pivot;

import static frc.robot.Constants.PivotConstants.PIVOT_ANGLES_MAP;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Pivot;
import org.littletonrobotics.junction.Logger;

public class AutoPivotAim extends Command {
  Pivot mPivot;
  Cameras mCamera;
  double errorThreshold = 0.15;
  double distanceToAprilTag = 0.0;
  double angleToSet = 0.0;

  public AutoPivotAim(Pivot pivot, Cameras camera) {
    mCamera = camera;
    mPivot = pivot;
    addRequirements(pivot);
  }

  @Override
  public void execute() {
    if (mCamera.hasTarget()) {
      distanceToAprilTag = mCamera.getPrimaryAprilTagPose().getZ();
      angleToSet = PIVOT_ANGLES_MAP.get(distanceToAprilTag);
      if (distanceToAprilTag != 0.0) mPivot.setPivotAngle(angleToSet);
    }
    Logger.recordOutput("AutoPivotAim angle", angleToSet);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(mPivot.getPivotAngle() - PIVOT_ANGLES_MAP.get(distanceToAprilTag))
        < errorThreshold;
  }
}
