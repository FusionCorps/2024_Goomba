package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexConstants;
import frc.robot.subsystems.Index;

// runs index, regardless of shoot speeds - used in auto
public class ShootAuto extends Command {
  Index mIndex;

  public ShootAuto(Index index) {
    mIndex = index;
    addRequirements(mIndex);
  }

  @Override
  public void execute() {
    System.out.println("I am here");
    mIndex.runIndex(IndexConstants.INDEX_RUN_PCT);
  }

  @Override
  public boolean isFinished() {
    return !mIndex.beamBroken();
  }

  @Override
  public void end(boolean isFinished) {
    mIndex.runIndex(0);
  }
}
