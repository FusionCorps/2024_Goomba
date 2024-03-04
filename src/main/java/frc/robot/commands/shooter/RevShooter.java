package frc.robot.commands.shooter;

import static frc.robot.Constants.IndexConstants.IS_TRAPPING;
import static frc.robot.Constants.ShooterConstants.IS_AMP;
import static frc.robot.Constants.ShooterConstants.IS_SHOOTING_RIGHT;
import static frc.robot.Constants.ShooterConstants.SHUTTLING_RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
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

    if (!IS_TRAPPING) {
      if (!PivotConstants.IS_SHUTTLING) {
        if (!IS_AMP) {
          if (!IS_SHOOTING_RIGHT) {
            mShooter.setRPMs(lRPM, rRPM);
          } else {
            mShooter.setRPMs(rRPM, lRPM);
          }
        } else {
          mShooter.setRPMs(0, 0);
        }
      } else {
        mShooter.setRPMs(SHUTTLING_RPM, SHUTTLING_RPM);
      }
    } else {
      mShooter.setRPMs(0, 0);
    }

  }

  // @Override
  // public void end(boolean interrupted) {
  // mShooter.setRPMs(0, 0);
  // ShooterConstants.IS_AMP = false;
  // }
}
