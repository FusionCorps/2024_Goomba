package frc.robot.commands.TransferHooks;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransferHooks;

public class HoldHooks extends Command {

  TransferHooks mTransferHooks;
  double pos;

  public HoldHooks(TransferHooks transferHooks) {
    mTransferHooks = transferHooks;
    addRequirements(mTransferHooks);
  }

  @Override
  public void execute() {
    System.out.println("I'm here");
    mTransferHooks.holdAtTargetPos();
  }

  @Override
  public boolean isFinished(){
    return true;
  }
}
