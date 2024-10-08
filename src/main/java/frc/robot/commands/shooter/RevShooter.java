package frc.robot.commands.shooter;

import static frc.robot.Constants.IndexConstants.IS_TRAPPING;
import static frc.robot.Constants.PivotConstants.IS_SHUTTLING;
import static frc.robot.Constants.ShooterConstants.AMP_LEFT_SPEED;
import static frc.robot.Constants.ShooterConstants.AMP_RIGHT_SPEED;
import static frc.robot.Constants.ShooterConstants.HAS_STOPPED_REVVING;
import static frc.robot.Constants.ShooterConstants.IS_AMP;
import static frc.robot.Constants.ShooterConstants.IS_SHOOTING_RIGHT;
import static frc.robot.Constants.ShooterConstants.LEFT_SHUTTLING_RPM;
import static frc.robot.Constants.ShooterConstants.RIGHT_SHUTTLING_RPM;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

/** Runs the shooter at given RPMS (or preset if in certain situations). */
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
  public void initialize() {
    HAS_STOPPED_REVVING = false;
  }

  @Override
  public void execute() {
    SmartDashboard.putString(
        "Shooter Consts", IS_AMP + ", " + IS_SHUTTLING + ", " + IS_TRAPPING + ", " + IS_TRAPPING);

    HAS_STOPPED_REVVING = false;

    if (mShooter.isOuttaking.getBoolean(true)) {
      mShooter.setRPMs(-1000, -1000);
    } else if (mShooter.isShooting.getBoolean(true)) {
      mShooter.setRPMs(2000, 2000);
    } else if (IS_TRAPPING) {
      mShooter.setRPMs(0, 0);
    } else if (IS_SHUTTLING) {
      mShooter.setRPMs(LEFT_SHUTTLING_RPM, RIGHT_SHUTTLING_RPM);
    } else if (IS_AMP) {
      mShooter.setRPMs(AMP_LEFT_SPEED, AMP_RIGHT_SPEED);
    } else if (IS_SHOOTING_RIGHT) {
      mShooter.setRPMs(rRPM, lRPM);
    } else {
      mShooter.setRPMs(lRPM, rRPM);
    }

    if (!DriverStation.isAutonomous()) {
      if (mShooter.reachedSpeeds()) {
        RobotContainer.robotController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.1);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.robotController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }
}
