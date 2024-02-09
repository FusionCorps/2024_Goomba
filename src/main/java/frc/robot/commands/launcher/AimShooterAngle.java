package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class AimShooterAngle extends Command {
  private Pivot mPivot;
  private double pct;

  public AimShooterAngle(Pivot pivot, double angle){
      pct = angle;
      mPivot = pivot;

      addRequirements(mPivot);
  }

  @Override
  public void execute(){
      mPivot.setShooterAngle(pct);
  }

  @Override
  public void end(boolean isInterrupted){
    mPivot.setShooterAngle(0);
  }

}
