package frc.robot.commands;

import static frc.robot.Constants.PivotConstants.PIVOT_CLIMB_DOWN_POS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TransferHookConstants;
import frc.robot.commands.TransferHooks.HoldHooks;
import frc.robot.commands.TransferHooks.SetHooksPos;
import frc.robot.commands.pivot.HoldPivotAngle;
import frc.robot.commands.pivot.SetPivotPos;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.TransferHooks;

public class Climb extends SequentialCommandGroup {
  public Climb(Pivot pivot, TransferHooks transferHooks, Index index) {
    addCommands(
        new SetPivotPos(pivot, PIVOT_CLIMB_DOWN_POS),
        new HoldPivotAngle(pivot)
            .alongWith(
                new SetHooksPos(transferHooks, TransferHookConstants.TRANSFER_HOOK_POS_CLIMB)),
        new HoldHooks(transferHooks));
  }
}
