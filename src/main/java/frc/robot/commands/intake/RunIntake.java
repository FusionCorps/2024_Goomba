package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  private Intake m_intake;
  double speed;

  public RunIntake(Intake intake, double pct) {
    m_intake = intake;
    speed = pct;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    m_intake.runIntake(speed);
  }

  @Override
  public void end(boolean isInterrupted) {
    m_intake.runIntake(0);
  }
}
