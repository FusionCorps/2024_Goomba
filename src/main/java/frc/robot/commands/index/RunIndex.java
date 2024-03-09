package frc.robot.commands.index;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
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
    if(mIndex.beamBroken()){
      RobotContainer.robotController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.1);
    }
  }

  // @Override
  // public boolean isFinished() {
  //   return mIndex.beamBroken();
  // }

  @Override
  public void end(boolean isInterrupted) {
    RobotContainer.robotController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    mIndex.runIndex(0);
  }
}
