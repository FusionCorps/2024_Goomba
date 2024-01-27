package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;

public class RunIndex extends Command {

    Index mIndex;
    double pct;


    public RunIndex(Index index, double pwr){
        mIndex = index;
        pct = pwr;

        addRequirements(mIndex);
    }

    @Override
    public void execute(){

        mIndex.indexIn(pct);
    }

    @Override
    public void end(boolean interrupted){
        mIndex.indexIn(0);
    }
    
}
