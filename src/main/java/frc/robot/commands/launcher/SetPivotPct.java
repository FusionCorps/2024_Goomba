package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class SetPivotPct extends Command {
  private Pivot mPivot;
  private double pct;

  public SetPivotPct(Pivot pivot, double pct) {
    this.pct = pct;
    mPivot = pivot;

    addRequirements(mPivot);
  }

  @Override
  public void execute() {
    mPivot.setPivotPct(pct);
  }

  @Override
  public void end(boolean isInterrupted) {
    // mPivot.stabilizeMotors();
    mPivot.setPivotPct(0);
  }
}
