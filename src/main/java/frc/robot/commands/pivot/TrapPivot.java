package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class TrapPivot extends Command {

  Pivot mPivot;
  double angle;

  public TrapPivot(Pivot pivot, double ang) {
    mPivot = pivot;
    angle = ang;

    addRequirements(mPivot);
  }

  @Override
  public void execute() {
    mPivot.setPivotAngle(angle, 500, 100, 1000);
  }

  @Override
  public boolean isFinished() {
    return mPivot.reachedAngle();
  }
}
