package frc.robot.commands.swerve.manual;

import static frc.robot.Constants.DrivetrainConstants.AimingDamper;
import static frc.robot.Constants.DrivetrainConstants.DriveDeadband;
import static frc.robot.Constants.DrivetrainConstants.MaxSpeed;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

/** Runs the swerve drive in field centric mode, locks the robot's heading and moves more slowly */
// UNUSED, UNTESTED
public class RunSwerveFCWithAim extends Command {
  private Drivetrain mDrivetrain;
  private CommandXboxController controller = RobotContainer.robotController;

  SwerveRequest.FieldCentricFacingAngle req = new SwerveRequest.FieldCentricFacingAngle();

  public RunSwerveFCWithAim(Drivetrain drivetrain) {
    mDrivetrain = drivetrain;
    addRequirements(mDrivetrain);
  }

  @Override
  public void initialize() {
    req =
        req.withTargetDirection(
            Rotation2d.fromDegrees(mDrivetrain.getPigeon2().getYaw().getValue() % 360.0));
  }

  @Override
  public void execute() {
    mDrivetrain.setControl(
        req.withVelocityX(-controller.getLeftY() * MaxSpeed * AimingDamper)
            .withVelocityY(-controller.getLeftX() * MaxSpeed * AimingDamper)
            .withDeadband(DriveDeadband));
  }
}
