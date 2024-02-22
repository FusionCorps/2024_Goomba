package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

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
    // if (beamBreak.get()) {

    //   mPivot.setAngle(24.51);
    // } else {
    //   mPivot.setAngle(PivotConstants.PIVOT_OFFSET * PivotConstants.PIVOT_GEAR_RATIO);
    // }
    mPivot.setPivotAngle(angle);
  }
}
