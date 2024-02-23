package frc.robot.commands.launcher;

import static frc.robot.Constants.PivotConstants.PIVOT_ANGLES_MAP;
import static frc.robot.Constants.diagnosticsTab;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;

public class AutoPivotAim extends Command {
  Pivot mPivot;
  Drivetrain mDrivetrain;
  double errorThreshold = 0.15;
  double distanceToAprilTag = 0.0;
  double angleToSet = 0.0;

  public AutoPivotAim(Drivetrain drivetrain, Pivot pivot) {
    mDrivetrain = drivetrain;
    mPivot = pivot;
    addRequirements(pivot);

    diagnosticsTab.addDouble("AutoPivotAim angle", () -> angleToSet);
  }

  @Override
  public void execute() {
    if (mDrivetrain.getCamera().hasTarget()) {
      distanceToAprilTag = mDrivetrain.getCamera().getPrimaryAprilTagPose().getZ();
      angleToSet = PIVOT_ANGLES_MAP.get(distanceToAprilTag);
      if (distanceToAprilTag != 0.0) mPivot.setPivotAngle(angleToSet);
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(mPivot.getPivotAngle() - PIVOT_ANGLES_MAP.get(distanceToAprilTag))
        < errorThreshold;
  }
}
