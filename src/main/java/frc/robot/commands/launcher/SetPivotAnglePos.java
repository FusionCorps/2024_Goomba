package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class SetPivotAnglePos extends Command {

  Pivot mPivot;
  double angle;

  public SetPivotAnglePos(Pivot pivot, double ang) {
    mPivot = pivot;
    angle = ang;

    addRequirements(mPivot);
  }

  @Override
  public void execute() {
    mPivot.setAngle(angle);
  }
}
