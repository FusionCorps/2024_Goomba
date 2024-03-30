package frc.robot.commands.pivot;

import static frc.robot.Constants.IndexConstants.IS_TRAPPING;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Pivot;

public class SetPivotPct extends Command {
  private Pivot mPivot;
  private Index mIndex;
  private double pct;

  public SetPivotPct(Pivot pivot, Index index, double pct) {
    this.pct = pct;
    mPivot = pivot;
    mIndex = index;

    addRequirements(mPivot, mIndex);
  }

  @Override
  public void initialize() {
    IS_TRAPPING = true;
  }

  @Override
  public void execute() {
    mPivot.setPivotPct(pct);
    mIndex.runIndex(0.06);
  }

  @Override
  public void end(boolean isInterrupted) {
    // mPivot.stabilizeMotors();
    mPivot.setPivotPct(0);
    mIndex.runIndex(0);
  }
}
