package frc.robot.commands.index;

import static frc.robot.Constants.IndexConstants.INDEX_AMP_PCT;
import static frc.robot.Constants.IndexConstants.INDEX_RUN_PCT;
import static frc.robot.Constants.IndexConstants.IS_TRAPPING;
import static frc.robot.Constants.ShooterConstants.HAS_STOPPED_REVVING;
import static frc.robot.Constants.ShooterConstants.IS_AMP;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;

public class IndexDummy extends Command {
  public Index mIndex;

  public IndexDummy(Index index) {
    mIndex = index;
    addRequirements(mIndex);
  }

  @Override
  public void execute() {
    if(mIndex.isOuttaking.getBoolean(true)){
      mIndex.runIndex(-INDEX_RUN_PCT);
    } else if(mIndex.isIndexing.getBoolean(true)){
      mIndex.runIndex(INDEX_RUN_PCT);
    }else if (HAS_STOPPED_REVVING) {
      mIndex.runIndex(0.0);
    } else if (IS_TRAPPING) {
      mIndex.runIndex(-0.22);
    } else if (IS_AMP) {
      mIndex.runIndex(INDEX_AMP_PCT);
    } else {
      mIndex.runIndex(INDEX_RUN_PCT);
    }
  }

  @Override
  public void end(boolean isFinished) {
    mIndex.runIndex(0);
  }
}
