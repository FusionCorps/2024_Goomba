package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class ResetShooterAngle extends Command {

  Pivot mPivot;

  public ResetShooterAngle(Pivot pivot) {
    mPivot = pivot;

    addRequirements(mPivot);
  }

  @Override
  public void execute() {
    mPivot.resetShooterAngle();
  }
}
