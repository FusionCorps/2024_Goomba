package frc.robot.commands;

import static frc.robot.Constants.PivotConstants.PIVOT_CLIMB_DOWN_POS;
import static frc.robot.Constants.PivotConstants.PIVOT_CLIMB_UP_POS;
import static frc.robot.Constants.PivotConstants.PIVOT_TRAP_POS;
import static frc.robot.Constants.TransferHookConstants.TRANSFER_HOOK_POS_CLIMB;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TransferHooks.SetHooksPos;
import frc.robot.commands.index.RunIndex;
import frc.robot.commands.pivot.SetPivotPos;
import frc.robot.commands.swerve.manual.RunSwerveRC;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.TransferHooks;

/**
 * Climb routine. Raise the pivot, move the robot forward a little, lower the pivot back down over
 * the chain, move the hooks, move the pivot to position for trap scoring, and run the index
 */
public class Climb extends SequentialCommandGroup {
  public Climb(Pivot pivot, Drivetrain drivetrain, TransferHooks hooks, Index index) {
    addCommands(
        new SetPivotPos(pivot, PIVOT_CLIMB_UP_POS)
            .alongWith(new RunSwerveRC(drivetrain, 0.005, 0, 0).withTimeout(0.2))
            .andThen(new SetPivotPos(pivot, PIVOT_CLIMB_DOWN_POS))
            .andThen(new SetHooksPos(hooks, TRANSFER_HOOK_POS_CLIMB))
            .andThen(new SetPivotPos(pivot, PIVOT_TRAP_POS))
            .andThen(new RunIndex(index, -0.23)));
  }
}
