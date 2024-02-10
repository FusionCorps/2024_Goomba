package frc.robot.commands.cobra;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cobra;

public class SetCobraPct extends Command {

  double mPct;
  Cobra mCobra;

  public SetCobraPct(Cobra cobra, double pct) {
    mCobra = cobra;
    mPct = pct;
    addRequirements(mCobra);
  }

  @Override
  public void execute() {
    mCobra.setCobraPct(mPct);
  }

  @Override
  public void end(boolean isInterrupted) {
    mCobra.setCobraPct(0);
  }
}
