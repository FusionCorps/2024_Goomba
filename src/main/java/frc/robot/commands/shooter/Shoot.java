package frc.robot.commands.shooter;

import static frc.robot.Constants.IndexConstants.INDEX_PCT;
import static frc.robot.Constants.IndexConstants.IS_TRAPPING;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.subsystems.Index;
// import frc.robot.subsystems.Shooter;

// /** Runs the shooter at given RPMS. */
// public class Shoot extends Command {
//   private Shooter mShooter;

//   private Index mIndex;
//   // Timer mTimer = new Timer();

//   // boolean hasTimerStarted = false;

//   private double lRPM = 0;
//   private double rRPM = 0;

//   public Shoot(Shooter shooter, Index index, double lRPM, double rRPM) {
//     mShooter = shooter;
//     mIndex = index;
//     this.lRPM = lRPM;
//     this.rRPM = rRPM;

//     addRequirements(mShooter);
//   }

//   @Override
//   public void initialize() {
//     // mTimer.stop();
//     // mTimer.reset();
//   }

//   @Override
//   public void execute() {
//     mShooter.setRPMs(lRPM, rRPM);
//   }

//   // @Override
//   // public boolean isFinished() {

//   //   // if (!mIndex.beamBroken()) {

//   //   //   // mTimer.start();
//   //   //   // hasTimerStarted = true;
//   //   // }

//   //   // // if (hasTimerStarted && mTimer.hasElapsed(0.5)) {
//   //   // //   mTimer.stop();
//   //   // //   mTimer.reset();
//   //   //   return true;
//   //   // }

//   //   return false;
//   // }

//   @Override
//   public void end(boolean interrupted) {
//     mShooter.setRPMs(0, 0);

//     // mShooter.shootRightRPM(0);
//     ShooterConstants.IS_AMP = false;
//   }
// }

public class Shoot extends Command {
  public Shooter mShooter;
  public Index mIndex;

  public Shoot(Shooter shooter, Index index) {
    mIndex = index;
    mShooter = shooter;
    addRequirements(mIndex);
  }

  @Override
  public void execute() {
    if (!IS_TRAPPING) {
      if (mIndex.beamBroken() && mShooter.reachedSpeeds()) {
        mIndex.runIndex(INDEX_PCT);
      }
    } else{
      mIndex.runIndex(-INDEX_PCT);
    }
  }

  @Override
  public boolean isFinished() {
    return !mIndex.beamBroken();
  }

  @Override
  public void end(boolean interrupted) {
    mIndex.runIndex(0.0);
    IS_TRAPPING = false;
  }
}
