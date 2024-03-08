package frc.robot.commands.shooter;

import static frc.robot.Constants.IndexConstants.INDEX_PCT;
import static frc.robot.Constants.IndexConstants.IS_TRAPPING;
import static frc.robot.Constants.ShooterConstants.HAS_STOPPED_REVING;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  public Shooter mShooter;
  public Index mIndex;

  public Shoot(Shooter shooter, Index index) {
    mIndex = index;
    mShooter = shooter;
    addRequirements(mIndex);
  }

  @Override
  public void execute() {
    if (!HAS_STOPPED_REVING) {
      if (!IS_TRAPPING) {
        // if (mIndex.beamBroken() && mShooter.reachedSpeeds()) {
          mIndex.runIndex(INDEX_PCT);
        // }
      } else {
        mIndex.runIndex(-INDEX_PCT);
      }
    } else {
      mIndex.runIndex(0);
    }

  }

  // @Override
  // public boolean isFinished() {
  //   return !mIndex.beamBroken();
  // }

  @Override
  public void end(boolean interrupted) {
    mIndex.runIndex(0.0);
    IS_TRAPPING = false;
  }
}
