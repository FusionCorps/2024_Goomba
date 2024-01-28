package frc.robot.commands.swerve.manual;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Lock wheels in X shape */
public class SwerveBrake extends Command {
  private CommandSwerveDrivetrain mDrivetrain;

  public SwerveBrake(CommandSwerveDrivetrain drivetrain) {
    mDrivetrain = drivetrain;
    addRequirements(mDrivetrain);
  }

  @Override
  public void execute() {
    mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
  }
}
