package frc.robot.commands.pivot;

import static frc.robot.Constants.PivotConstants.PIVOT_TRAP_POS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TransferHookConstants;
import frc.robot.commands.TransferHooks.SetHooksPos;
import frc.robot.commands.index.RunIndex;
import frc.robot.commands.swerve.manual.RunSwerveRC;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.TransferHooks;

public class Climb extends SequentialCommandGroup {

    public Climb(Pivot pivot, Drivetrain drivetrain, TransferHooks hooks, Index index) {
        addCommands(new SetPivotPos(pivot, PivotConstants.PIVOT_CLIMB_UP_POS)
                .alongWith(new RunSwerveRC(drivetrain, 0.005, 0, 0).withTimeout(0.2)).andThen(new DownClimbPos(pivot))
                .andThen(new SetHooksPos(hooks, TransferHookConstants.TRANSFER_HOOK_POS_CLIMB))
                .andThen(new SetPivotPos(pivot, PIVOT_TRAP_POS)).andThen(new RunIndex(index, -0.23)));
    }

}
