package frc.robot.commands.TransferHooks;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransferHooks;

public class SetHooksPos extends Command {

  TransferHooks mTransferHooks;
  double pos;

  public SetHooksPos(TransferHooks transferHooks, double position) {
    mTransferHooks = transferHooks;
    pos = position;

    addRequirements(mTransferHooks);
  }

  @Override
  public void execute() {
    mTransferHooks.setHookPos(pos);
  }

  @Override
  public boolean isFinished() {
    return mTransferHooks.reachedPos();
  }
}
