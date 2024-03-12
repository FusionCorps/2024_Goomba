package frc.robot.commands.pivot;

import static frc.robot.Constants.PivotConstants.PIVOT_AMP_POS;
import static frc.robot.Constants.ShooterConstants.IS_AMP;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class SetAngleAmp extends Command {

  Pivot mPivot;

  public SetAngleAmp(Pivot pivot) {
    mPivot = pivot;
  }

  @Override
  public void initialize() {
    IS_AMP = true;
  }

  @Override
  public void execute() {
    mPivot.setPivotAngle(PIVOT_AMP_POS);
  }

  @Override
  public boolean isFinished() {
    return mPivot.reachedAngle();
  }
}
