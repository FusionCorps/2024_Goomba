package frc.robot.commands.launcher;

import static frc.robot.Constants.PivotConstants.PIVOT_ANGLES_MAP;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;

public class AutoPivotAim extends Command {
  Pivot mPivot;
  Drivetrain mDrivetrain;

  public AutoPivotAim(Drivetrain drivetrain, Pivot pivot) {
    mDrivetrain = drivetrain;
    mPivot = pivot;
    addRequirements(mDrivetrain, pivot);
  }

  @Override
  public void execute() {
    double distanceToAprilTag = mDrivetrain.getCamera().getPrimaryAprilTagPose().getZ();
    mPivot.setAngle(PIVOT_ANGLES_MAP.get(distanceToAprilTag));
  }
}
