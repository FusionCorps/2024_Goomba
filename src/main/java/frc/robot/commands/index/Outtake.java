package frc.robot.commands.index;

import static frc.robot.Constants.IndexConstants.INDEX_RUN_PCT;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;

public class Outtake extends Command {

  Index mIndex;

  public Outtake(Index index) {
    mIndex = index;

    addRequirements(mIndex);
  }

  @Override
  public void execute() {
    mIndex.runIndex(-INDEX_RUN_PCT);
  }

  @Override
  public void end(boolean isInterrupted) {
    mIndex.runIndex(0);
  }
}
