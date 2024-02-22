package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  private Intake mIntake;

  double speed;

  public RunIntake(Intake intake, double pct) {
    mIntake = intake;
    speed = pct;

    addRequirements(intake);
  }

  @Override
  public void execute() {
    mIntake.runIntake(speed);
  }

  @Override
  public void end(boolean isInterrupted) {
    mIntake.runIntake(0);
  }
}
