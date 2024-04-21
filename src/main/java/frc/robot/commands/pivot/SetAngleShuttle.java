package frc.robot.commands.pivot;

import static frc.robot.Constants.PivotConstants.IS_SHUTTLING;
import static frc.robot.Constants.PivotConstants.PIVOT_SHUTTLING_POS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

// sets pivot angle to shuttle position
public class SetAngleShuttle extends Command {
  Pivot mPivot;

  public SetAngleShuttle(Pivot pivot) {
    mPivot = pivot;
    addRequirements(mPivot);
  }

  @Override
  public void initialize() {
    IS_SHUTTLING = true;
  }

  @Override
  public void execute() {
    mPivot.setPivotAngle(PIVOT_SHUTTLING_POS);
  }
}
