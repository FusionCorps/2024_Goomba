package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;

public class RunIndex extends Command {
  Index mIndex;
  double pct;

  public RunIndex(Index index, double pct) {
    mIndex = index;
    this.pct = pct;

    addRequirements(mIndex);
  }

  @Override
  public void execute() {
    mIndex.runIndex(pct);
  }

  @Override
  public boolean isFinished() {
    return mIndex.beamBroken();
  }

  @Override
  public void end(boolean isInterrupted) {
    mIndex.runIndex(0);
  }
}
