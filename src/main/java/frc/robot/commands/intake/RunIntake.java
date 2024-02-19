package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import java.util.function.BooleanSupplier;

public class RunIntake extends Command {
  private Intake mIntake;
  BooleanSupplier beamBroken;
  double speed;

  public RunIntake(Intake intake, double pct, BooleanSupplier beamBroken) {
    mIntake = intake;
    speed = pct;
    this.beamBroken = beamBroken;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    if (!beamBroken.getAsBoolean()) mIntake.runIntake(speed);
  }

  @Override
  public void end(boolean isInterrupted) {
    mIntake.runIntake(0);
  }
}
