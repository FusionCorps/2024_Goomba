package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AimShooterAngle extends Command {
  private Shooter mShooter;
  private double pct;

  public AimShooterAngle(Shooter shooter, double angle) {
    pct = angle;
    mShooter = shooter;

    addRequirements(mShooter);
  }

  @Override
  public void execute() {
    mShooter.setShooterAngle(pct);
  }

  @Override
  public void end(boolean isInterrupted) {
    mShooter.setShooterAngle(0);
  }
}
