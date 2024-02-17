package frc.robot.commands.swerve.vision;

import static frc.robot.Constants.DrivetrainConstants.MaxAngularRate;
import static frc.robot.Constants.LimelightConstants.LIMELIGHT_TX_RANGE_DEG;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimingPIDS;
import frc.robot.subsystems.Drivetrain;

// aims at a target in place
public class AimAtTarget extends Command {
  private Drivetrain mDrivetrain;
  private PIDController pid =
      new PIDController(
          AimingPIDS.toTargetRotKP, AimingPIDS.toTargetRotKI, AimingPIDS.toTargetRotKD);
  private Timer timer = new Timer();
  private double runTime = 0.0;

  public AimAtTarget(Drivetrain drivetrain, double toleranceDeg, double runTime) {
    pid.setTolerance(toleranceDeg);
    pid.setSetpoint(0.0); // goal is to have tx be 0 (centered on target)
    this.runTime = runTime;
    mDrivetrain = drivetrain;
    addRequirements(mDrivetrain);

    timer.start();
  }

  @Override
  public void execute() {
    // if target detected, rotate to target
    if (mDrivetrain.getCamera().hasTarget() && mDrivetrain.getCamera().getID() == 4) {
      // get pid output of normalized tx (-1 to 1) and scale by max angular rate
      SwerveRequest req =
          new SwerveRequest.FieldCentric()
              .withRotationalRate(
                  pid.calculate(mDrivetrain.getCamera().getTX() / LIMELIGHT_TX_RANGE_DEG)
                      * MaxAngularRate);

      mDrivetrain.setControl(req);
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(runTime);
  }

  @Override
  public void end(boolean interrupted) {
    // stop rotating
    mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    pid.close();
    timer.stop();
    timer.reset();
  }
}
