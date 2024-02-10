package frc.robot.commands.swerve.manual;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class RotateInStage extends SequentialCommandGroup {

  Drivetrain mDrivetrain;

  public RotateInStage(Drivetrain drivetrain) {
    mDrivetrain = drivetrain;

    addRequirements(mDrivetrain);
  }
}
