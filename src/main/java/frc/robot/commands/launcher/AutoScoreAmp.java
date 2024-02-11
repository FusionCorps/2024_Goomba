package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.RunIndex;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;


/* Command that runs automatically when robot gets close enough to amp 
 * Specifically, it:
 *  1) Moves pivot to amp position and runs shooter to target velocity
 *  2) After both of those targets are achieved, index note to shooter for a few seconds
*/
public class AutoScoreAmp extends SequentialCommandGroup {

    Shooter mShooter;
    Pivot mPivot;
    Index mIndex;

    public AutoScoreAmp(Shooter shooter, Pivot pivot, Index index) {

        mShooter = shooter;
        mPivot = pivot;
        mIndex = index;

        addRequirements(mShooter);
        addRequirements(mPivot);
        addRequirements(mIndex);

        addCommands(
            new SetPivotPos(pivot, 0)
            .alongWith(new Shoot(mShooter, ShooterConstants.AMP_LEFT_SPEED, ShooterConstants.AMP_RIGHT_SPEED))
            .until(() -> mShooter.reachedSpeeds() && mPivot.reachedAngle())
            .andThen(new RunIndex(mIndex, IndexConstants.INDEX_PCT))
            .withTimeout(4)
        );
        
    }
    
}
