package frc.robot.commands.tarsarm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TarsArm;

public class SetBasePos extends Command {

  TarsArm m_arm;
  double pos;

  public SetBasePos(TarsArm arm, double armPos) {
    m_arm = arm;
    pos = armPos;
    addRequirements(m_arm);
  }

  @Override
  public void execute() {
    m_arm.setBasePos(pos);
  }
}
