package frc.robot.commands;

import static frc.robot.Constants.PivotConstants.PIVOT_ANGLES_MAP;
import static frc.robot.Constants.ShooterConstants.ShooterSpeed;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AimingPIDS;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

// ------UNUSED, UNTESTED------
public class ShootOnMove extends Command {
  CommandXboxController controller = RobotContainer.robotController;
  Drivetrain mDrivetrain;
  Shooter mShooter;
  Pivot mPivot;

  double tolerance = 0.0;

  SwerveRequest.FieldCentricFacingAngle req;

  public ShootOnMove(Drivetrain drivetrain, Shooter shooter, Pivot pivot, double tolerance) {
    mDrivetrain = drivetrain;
    mShooter = shooter;
    mPivot = pivot;
    this.tolerance = tolerance;

    req =
        new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(
                Rotation2d.fromDegrees(mDrivetrain.getPigeon2().getYaw().getValue() % 360.0));

    addRequirements(drivetrain, mShooter, mPivot);

    req.HeadingController.setPID(
        AimingPIDS.toAngleRotKP, AimingPIDS.toAngleRotKI, AimingPIDS.toAngleRotKD);
    req.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    req.HeadingController.setTolerance(Units.degreesToRadians(tolerance));
  }

  /**
   * Procedure: 1) Find the time it takes a note to reach the wall (timeToWall) 2) Use this time to
   * find the distance the note will travel horizontally (skewDistance) 3) Shoot when the current Z
   * distance to the wall = skewDistance
   */
  @Override
  public void execute() {
    Pose3d aprilTagPose = mDrivetrain.getCamera().getPrimaryAprilTagPose();

    mPivot.setPivotAngle(PIVOT_ANGLES_MAP.get(aprilTagPose.getZ()));

    double zVelocity = ShooterSpeed * Math.cos(mPivot.getPivotAngle());

    double timeToWall = aprilTagPose.getZ() / zVelocity;

    double skewDistance = timeToWall * mDrivetrain.getState().speeds.vyMetersPerSecond;

    if (Math.abs(aprilTagPose.getX() - skewDistance) < tolerance) {
      mShooter.setRPMs(0.8, 0.6);
    }

    mDrivetrain.setControl(req.withVelocityX(0).withVelocityY(-controller.getLeftX()));
  }
}
