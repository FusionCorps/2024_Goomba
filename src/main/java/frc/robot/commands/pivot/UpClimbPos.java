package frc.robot.commands.pivot;

import static frc.robot.Constants.IndexConstants.IS_TRAPPING;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

public class UpClimbPos extends Command {

  Pivot mPivot;

  public UpClimbPos(Pivot pivot) {
    mPivot = pivot;

    addRequirements(mPivot);
  }

  @Override
  public void initialize() {
    IS_TRAPPING = true;
  }

  @Override
  public void execute() {
    mPivot.setPivotAngle(PivotConstants.PIVOT_CLIMB_UP_POS);
  }

  @Override
  public boolean isFinished() {
    return mPivot.reachedAngle();
  }
}
