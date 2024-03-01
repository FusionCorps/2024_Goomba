package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

/** Runs the shooter at given RPMS. */
public class RevShooter extends Command {
  private Shooter mShooter;

  private double lRPM = 0;
  private double rRPM = 0;

  public RevShooter(Shooter shooter, double lRPM, double rRPM) {
    mShooter = shooter;
    this.lRPM = lRPM;
    this.rRPM = rRPM;

    addRequirements(mShooter);
  }

  @Override
  public void execute() {
    mShooter.setRPMs(lRPM, rRPM);
  }

  @Override
  public void end(boolean interrupted) {
    mShooter.setRPMs(0, 0);
    ShooterConstants.IS_AMP = false;
  }
}
