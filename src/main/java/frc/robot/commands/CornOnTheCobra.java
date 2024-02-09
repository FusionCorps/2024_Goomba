package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cobra;

public class CornOnTheCobra extends Command {

  Cobra mCobra;
  double pos;

  public CornOnTheCobra(Cobra cobra, double position) {
    mCobra = cobra;
    pos = position;

    addRequirements(mCobra);
  }

  @Override
  public void execute() {
    mCobra.setCobraPos(pos);
  }
}
