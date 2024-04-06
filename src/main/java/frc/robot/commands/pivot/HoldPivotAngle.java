package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class HoldPivotAngle extends Command {

  Pivot mPivot;

  public HoldPivotAngle(Pivot pivot) {
    mPivot = pivot;
    addRequirements(mPivot);
  }

  @Override
  public void execute() {
    System.out.println("holding pivot");
    mPivot.holdPivotPos();
  }

  @Override
  public boolean isFinished(){
    return true;
  }
}
