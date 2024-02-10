package frc.robot.commands;

import static frc.robot.Constants.shooterSpeed;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AimingPIDS;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class ShootOnMove extends Command {
  CommandXboxController mRobotController = RobotContainer.robotController;
  private static final double LENGTH_SPEAKER_EDGE = 0;
  private static final double h_s = 0;
  Drivetrain mDrivetrain;
  Shooter mShooter;

  Translation2d robotVel = new Translation2d();
  double distToAprilTagHorizontal = 0.0;
  double distToAprilTagVertical = 0.0;

  double goalTX = 0.0;
  double currTX = 0.0;

  double tolerance = 0.0;

  SwerveRequest.FieldCentricFacingAngle req;

  public ShootOnMove(Drivetrain drivetrain, Shooter shooter, double tolerance) {
    mDrivetrain = drivetrain;
    mShooter = shooter;
    this.tolerance = tolerance;

    req =
        new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(
                Rotation2d.fromDegrees(mDrivetrain.getPigeon2().getAngle() % 360.0));

    addRequirements(drivetrain, mShooter);

    req.HeadingController.setPID(
        AimingPIDS.toAngleRotKP, AimingPIDS.toAngleRotKI, AimingPIDS.toAngleRotKD);
    req.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    req.HeadingController.setTolerance(Units.degreesToRadians(tolerance));
  }

  /**
   * Procedure: Givens: shooter velocity, robot velocity 1. Get the robot's distance to an apriltag
   * 2. Based on distance, find
   */
  @Override
  public void execute() {

    robotVel = mDrivetrain.getVelocity();

    distToAprilTagHorizontal = mDrivetrain.getCamera().getDistToAprilTagHorizontal();
    distToAprilTagVertical = mDrivetrain.getCamera().getDistToAprilTagVertical();
    currTX = mDrivetrain.getCamera().getTX();

    calcTargetTXSimple();
    // calcTargetTXAdvanced();

    if (Math.abs(goalTX - currTX) < tolerance) {
      mShooter.shoot(0.8, 0.6);
    }

    mDrivetrain.setControl(
        req.withVelocityX(-mRobotController.getLeftY())
            .withVelocityY(-mRobotController.getLeftX()));
  }

  private void calcTargetTXSimple() {
    goalTX =
        Math.atan(
            (robotVel.getY() * Math.sin(Units.degreesToRadians(14)))
                / shooterSpeed
                * Math.cos(45 - currTX));
  }

  private void calcTargetTXAdvanced() {
    goalTX =
        45
            - Math.atan(
                (distToAprilTagVertical + LENGTH_SPEAKER_EDGE * Math.sin(14))
                    / (distToAprilTagHorizontal
                        + h_s * Math.tan(currTX + 45)
                        - LENGTH_SPEAKER_EDGE * Math.cos(14)));
  }
}
