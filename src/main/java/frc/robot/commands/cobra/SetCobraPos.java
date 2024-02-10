package frc.robot.commands.cobra;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cobra;

public class SetCobraPos extends Command {
  Cobra mCobra;
  double pos;

  public SetCobraPos(Cobra cobra, double position) {
    mCobra = cobra;
    pos = position;
    addRequirements(mCobra);
  }

  @Override
  public void execute() {
    mCobra.setCobraPos(pos);
  }
}
