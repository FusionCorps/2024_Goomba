package frc.robot.commands.swerve.manual;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RotateInStage extends SequentialCommandGroup {

  CommandSwerveDrivetrain mDrivetrain;

  public RotateInStage(CommandSwerveDrivetrain drivetrain) {
    mDrivetrain = drivetrain;

    addRequirements(mDrivetrain);
  }
}
