package frc.robot.commands;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class LaunchNote extends Command {

    Shooter mShooter;


    double launchPower = 0;

    public LaunchNote(Shooter shooter, double power) {

        

        mShooter = shooter;
        launchPower = power;

        addRequirements(mShooter);

    }

    
    @Override
    public void execute() {
        mShooter.shoot(launchPower);
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.shoot(0);
    }
    
}
