package frc.robot.commands.launcher;

import static frc.robot.Constants.diagnosticsTab;
import static frc.robot.Constants.PivotConstants.PIVOT_ANGLES_MAP;
import static frc.robot.Constants.PivotConstants.PIVOT_GEAR_RATIO;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;

public class AutoPivotAim extends Command {
  Pivot mPivot;
  Drivetrain mDrivetrain;
  double errorThreshold = 0.75;
  Double distanceToAprilTag;

  public AutoPivotAim(Drivetrain drivetrain, Pivot pivot) {
    mDrivetrain = drivetrain;
    mPivot = pivot;
    addRequirements(mDrivetrain, pivot);
  }

  @Override
  public void execute() {
    if (mDrivetrain.getCamera().hasTarget()) {
      distanceToAprilTag = mDrivetrain.getCamera().getPrimaryAprilTagPose().getZ();
      mPivot.setAngle(PIVOT_ANGLES_MAP.get(distanceToAprilTag));
      System.out.println(PIVOT_ANGLES_MAP.get(distanceToAprilTag) + ", " + (mPivot.getPivotAngle() - PIVOT_ANGLES_MAP.get(distanceToAprilTag)));
    } else{
      distanceToAprilTag = null;
    }
  }

  @Override
  public boolean isFinished(){
    if(distanceToAprilTag != null){
      return Math.abs(mPivot.getPivotAngle() - PIVOT_ANGLES_MAP.get(distanceToAprilTag)) < errorThreshold;
    }

    return false;
  }
}
