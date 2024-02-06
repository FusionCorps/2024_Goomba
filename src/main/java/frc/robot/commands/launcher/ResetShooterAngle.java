package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ResetShooterAngle extends Command{

    Shooter mShooter;

    public ResetShooterAngle(Shooter shooter){
        mShooter = shooter;

        addRequirements(mShooter);
    }

    @Override
    public void execute(){
        mShooter.resetShooterAngle();
    }
    
}
