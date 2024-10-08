package frc.robot.commands.swerve.vision;

import static frc.robot.Constants.DrivetrainConstants.MaxSpeed;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants.PIPELINE;
import frc.robot.subsystems.Drivetrain;

// ------UNUSED, UNTESTED------
// A command for driving to a note until it's not in vision of the camera.
public class DriveToNote extends Command {
  private Drivetrain mDrivetrain;
  private double tolerance = -18.3; // in degrees
  private boolean isDone;

  public DriveToNote(Drivetrain drivetrain) {
    mDrivetrain = drivetrain;

    isDone = false;
    addRequirements(mDrivetrain);
  }

  @Override
  public void execute() {
    if (mDrivetrain.getCamera().hasTarget()
        && mDrivetrain.getCamera().getPipeline() == PIPELINE.NOTE.value) {
      double ty = mDrivetrain.getCamera().getTY();
      if (ty > tolerance) {
        SwerveRequest req = new SwerveRequest.RobotCentric().withVelocityX(-0.3 * MaxSpeed);
        mDrivetrain.setControl(req);
      } else isDone = true;
    }
  }

  @Override
  public boolean isFinished() {

    return isDone;
  }

  @Override
  public void end(boolean isFinished) {
    mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    isDone = false;
  }
}
