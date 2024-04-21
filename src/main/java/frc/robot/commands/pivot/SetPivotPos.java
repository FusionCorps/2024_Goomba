package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

// sets pivot to a desired angle
public class SetPivotPos extends Command {
  Pivot mPivot;
  double angle;

  public SetPivotPos(Pivot pivot, double ang) {
    mPivot = pivot;
    angle = ang;

    addRequirements(mPivot);
  }

  @Override
  public void execute() {
    mPivot.setPivotAngle(angle);
  }

  @Override
  public boolean isFinished() {
    return mPivot.reachedAngle();
  }
}
