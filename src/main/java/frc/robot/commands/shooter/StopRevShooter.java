package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.HAS_STOPPED_REVVING;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StopRevShooter extends Command {

  Shooter mShooter;

  public StopRevShooter(Shooter shooter) {
    mShooter = shooter;

    addRequirements(mShooter);
  }

  @Override
  public void execute() {
    HAS_STOPPED_REVVING = true;
    mShooter.setRPMs(0.0, 0.0);
  }
}
