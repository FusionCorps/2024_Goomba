package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;

public class Trap extends Command {

  Index mIndex;

  public Trap(Index index) {
    mIndex = index;

    addRequirements(mIndex);
  }

  @Override
  public void execute() {
    mIndex.runIndex(-0.18);
  }

  @Override
  public void end(boolean isInterrupted) {
    mIndex.runIndex(0);
  }
}
