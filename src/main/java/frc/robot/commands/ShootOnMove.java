package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class ShootOnMove extends Command {
  CommandSwerveDrivetrain mDrivetrain;
  Shooter mShooter;

  public ShootOnMove(CommandSwerveDrivetrain drivetrain, Shooter shooter) {
    mDrivetrain = drivetrain;
    mShooter = shooter;
    addRequirements(drivetrain, mShooter);
  }

  /**
   * Procedure: Givens: shooter velocity, robot velocity 1. Get the robot's distance to an apriltag
   * 2. Based on distance, find
   */
  @Override
  public void execute() {}
}
