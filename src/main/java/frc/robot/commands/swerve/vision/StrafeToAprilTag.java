package frc.robot.commands.swerve.vision;

import static frc.robot.Constants.DrivetrainConstants.MaxAngularRate;
import static frc.robot.Constants.LimelightConstants.LIMELIGHT_TX_RANGE_DEG;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimingPIDS;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;

// horizontally strafes to align with a target
// UNUSED, UNTESTED
public class StrafeToAprilTag extends Command {
  private final Drivetrain mDrivetrain;
  private double tx;
  private PIDController strPID =
      new PIDController(AimingPIDS.strKP, AimingPIDS.strKI, AimingPIDS.strKD);
  private double toleranceDeg;

  public StrafeToAprilTag(Drivetrain drivetrain, double toleranceDeg) {
    this.toleranceDeg = toleranceDeg;
    strPID.setTolerance(toleranceDeg); // strafe to within tolerance degrees of target
    strPID.setSetpoint(0.0); // goal is to have tx be 0 (centered on target)
    mDrivetrain = drivetrain;
    addRequirements(mDrivetrain);
  }

  // horizontally align with target if target is detected
  @Override
  public void execute() {
    // if target detected, strafe toward it
    ShooterConstants.IS_AMP = true;
    if (mDrivetrain.getCamera().hasTarget()) {
      System.out.println("Strafe error: " + strPID.getPositionError());
      double tx = mDrivetrain.getCamera().getTX();
      // normalize tx to be between -1 and 1, then scale by max angular rate
      // set strafe velocity to velY * scaling factor
      SwerveRequest req =
          new SwerveRequest.FieldCentric()
              .withVelocityY(strPID.calculate(tx / LIMELIGHT_TX_RANGE_DEG) * MaxAngularRate)
              .withVelocityX(0.0);
      mDrivetrain.setControl(req);
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(tx) <= toleranceDeg;
  }

  @Override
  public void end(boolean interrupted) {
    // stop strafing
    mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    strPID.close();
  }
}
