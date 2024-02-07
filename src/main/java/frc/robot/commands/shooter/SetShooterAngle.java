package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooterAngle extends Command {

  Shooter mShooter;
  double angle;

  public SetShooterAngle(Shooter shooter, double ang) {
    mShooter = shooter;
    angle = ang;

    addRequirements(mShooter);
  }

  @Override
  public void execute() {
    mShooter.setAngle(angle);
  }
}
