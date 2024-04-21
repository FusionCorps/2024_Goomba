package frc.robot.commands.shooter;

import static frc.robot.Constants.IndexConstants.INDEX_RUN_PCT;
import static frc.robot.Constants.IndexConstants.IS_TRAPPING;
import static frc.robot.Constants.ShooterConstants.IS_AMP;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

// Runs index to shoot notes, only after shooter has revved up
public class Shoot extends Command {
  public Index mIndex;
  public Shooter mShooter;

  public Shoot(Index index, Shooter shooter) {
    mIndex = index;
    mShooter = shooter;
    addRequirements(mIndex);
  }

  @Override
  public void execute() {
    if (mShooter.reachedSpeeds()) {
      mIndex.runIndex(INDEX_RUN_PCT);
    }
  }

  @Override
  public boolean isFinished() {
    if (IS_AMP || IS_TRAPPING) {
      return false;
    }
    return !mIndex.beamBroken();
  }

  @Override
  public void end(boolean interrupted) {
    mIndex.runIndex(0.0);
    IS_TRAPPING = false;
  }
}
