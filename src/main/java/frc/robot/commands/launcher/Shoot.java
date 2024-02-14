package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  private Shooter mShooter;

  private double lRpm = 0;
  private double rRpm = 0;

  public Shoot(Shooter shooter, double leftRpm, double rightRpm) {
    mShooter = shooter;
    lRpm = leftRpm;
    rRpm = rightRpm;

    addRequirements(mShooter);
  }

  @Override
  public void execute() {
    mShooter.shoot(lRpm, rRpm);
  }

  @Override
  public void end(boolean interrupted) {
    mShooter.shoot(0, 0);
    ShooterConstants.IS_AMP = false;  }
}
