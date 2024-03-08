package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

public class Shuttling extends Command {
  Pivot mPivot;

  public Shuttling(Pivot pivot) {
    mPivot = pivot;

    addRequirements(mPivot);
  }

  @Override
  public void execute() {
    mPivot.setPivotAngle(PivotConstants.PIVOT_SHUTTLING_POS);
    PivotConstants.IS_SHUTTLING = true;
  }
}
