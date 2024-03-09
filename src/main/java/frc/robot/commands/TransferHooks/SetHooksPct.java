package frc.robot.commands.TransferHooks;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransferHooks;

public class SetHooksPct extends Command{

    TransferHooks mTransferHooks;
    double percentage;

    public SetHooksPct(TransferHooks transferHooks, double pct){
        mTransferHooks = transferHooks;
        percentage = pct;

        addRequirements(transferHooks);
    }

    @Override
    public void execute(){
        mTransferHooks.runHookPct(percentage);
    }

    @Override
    public void end(boolean interrupted){
        mTransferHooks.runHookPct(0);
    }
    
}
