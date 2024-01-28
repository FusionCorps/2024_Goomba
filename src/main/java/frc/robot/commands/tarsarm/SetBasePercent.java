package frc.robot.commands.tarsarm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TarsArm;

public class SetBasePercent extends Command {

  TarsArm m_Arm;
  double pct;

  public SetBasePercent(TarsArm arm, double pct) {
    m_Arm = arm;
    this.pct = pct;
    addRequirements(m_Arm);
  }

  @Override
  public void execute() {
    m_Arm.setBasePct(pct);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_Arm.setBasePct(0);
  }
}
