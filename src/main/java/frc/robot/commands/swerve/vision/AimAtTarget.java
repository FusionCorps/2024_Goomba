package frc.robot.commands.swerve.vision;

import static frc.robot.Constants.DrivetrainConstants.MaxAngularRate;
import static frc.robot.Constants.LimelightConstants.LIMELIGHT_TX_RANGE_DEG;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AimingPIDS;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

// aims at a target in place
public class AimAtTarget extends Command {
  CommandXboxController mController = RobotContainer.robotController;
  private Drivetrain mDrivetrain;
  private PIDController pid =
      new PIDController(
          AimingPIDS.toTargetRotKP, AimingPIDS.toTargetRotKI, AimingPIDS.toTargetRotKD);

  public AimAtTarget(Drivetrain drivetrain, double toleranceDeg) {
    pid.setTolerance(toleranceDeg);
    pid.setSetpoint(0.0); // goal is to have tx be 0 (centered on target)
    mDrivetrain = drivetrain;
    addRequirements(mDrivetrain);
  }

  @Override
  public void execute() {
    // if target detected, rotate to target
    if (mDrivetrain.getCamera().hasTarget()) {
      // get pid output of normalized tx (-1 to 1) and scale by max angular rate
      SwerveRequest req =
          new SwerveRequest.FieldCentric()
              .withVelocityX(-mController.getLeftY())
              .withVelocityY(-mController.getLeftX())
              .withDeadband(DrivetrainConstants.DriveDeadband)
              .withRotationalRate(
                  pid.calculate(mDrivetrain.getCamera().getTX() / LIMELIGHT_TX_RANGE_DEG)
                      * MaxAngularRate);

      mDrivetrain.setControl(req);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // stop rotating
    mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    pid.close();
    // timer.stop();
    // timer.reset();
  }
}
