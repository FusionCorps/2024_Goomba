package frc.robot.commands.swerve.manual;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/** Lock wheels in X shape */
public class SwerveBrake extends Command {
  private Drivetrain mDrivetrain;

  public SwerveBrake(Drivetrain drivetrain) {
    mDrivetrain = drivetrain;
    addRequirements(mDrivetrain);
  }

  @Override
  public void execute() {
    mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
  }
}
