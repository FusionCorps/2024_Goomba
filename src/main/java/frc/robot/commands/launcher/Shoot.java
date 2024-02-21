package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  private Shooter mShooter;

  private Index mIndex;
  Timer mTimer = new Timer();

  boolean hasTimerStarted = false;

  private double lRpm = 0;
  private double rRpm = 0;

  public Shoot(Shooter shooter, Index index, double leftRpm, double rightRpm) {
    mShooter = shooter;
    mIndex = index;
    lRpm = leftRpm;
    rRpm = rightRpm;

    addRequirements(mShooter);
  }

  @Override
  public void initialize() {
    mTimer.stop();
    mTimer.reset();
  }

  @Override
  public void execute() {
    // mShooter.shoot(lRpm, rRpm);
    mShooter.shootRightRPM(rRpm);
  }

  @Override
  public boolean isFinished() {

    if (!mIndex.beamBroken() && !hasTimerStarted) {

      mTimer.start();
      hasTimerStarted = true;
    }

    if (hasTimerStarted && mTimer.hasElapsed(0.5)) {
      mTimer.stop();
      mTimer.reset();
      return true;
    }

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // mShooter.shoot(0, 0);

    mShooter.shootRightRPM(0);
    ShooterConstants.IS_AMP = false;
  }
}
