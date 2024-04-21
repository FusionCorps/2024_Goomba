package frc.robot.commands;

import static frc.robot.Constants.IndexConstants.INDEX_AMP_PCT;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.index.RunIndex;
import frc.robot.commands.pivot.SetAngleAmp;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.commands.swerve.manual.RunSwerveRC;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

// Routine for automatically scoring at amp
// moves pivot to amp position and revs shooter, then moves forward for 1/4 s and runs  index
// unused
public class AutoScoreAmp extends SequentialCommandGroup {
  public AutoScoreAmp(Drivetrain drivetrain, Shooter shooter, Pivot pivot, Index index) {
    addCommands(
        new SetAngleAmp(pivot)
            .alongWith(
                new RevShooter(shooter, 0, 0)
                    .alongWith(new RunSwerveRC(drivetrain, 0.05, 0.0, 0.0).withTimeout(0.25))
                    .andThen(new RunIndex(index, INDEX_AMP_PCT))
                    .withTimeout(4)));
  }
}
