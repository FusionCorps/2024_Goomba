package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends Command {
    Shooter mShooter;
    double lRPM;
    double rRPM;

    public ShootAmp(Shooter shooter, double leftRPM, double rightRPM){

        mShooter = shooter;

        rRPM = -rightRPM;
        lRPM = -leftRPM;
        addRequirements(mShooter);
    }

    @Override
    public void execute(){
        mShooter.shoot(lRPM,rRPM);
    }

    @Override
  public void end(boolean interrupted) {
    mShooter.shoot(0, 0);
  }

}
