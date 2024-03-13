package frc.robot.commands.swerve.vision;

import static frc.robot.Constants.DrivetrainConstants.DriveDeadband;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AimingPIDS;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class AimAtPoint extends Command {
  CommandXboxController controller = RobotContainer.robotController;
  Drivetrain mDrivetrain;
  Translation2d mTarget = new Translation2d();

  SwerveRequest.FieldCentricFacingAngle headingReq = new SwerveRequest.FieldCentricFacingAngle();

  public AimAtPoint(Drivetrain drivetrain, Translation2d target) {
    mDrivetrain = drivetrain;
    mTarget = target;
    addRequirements(mDrivetrain);
    headingReq.HeadingController.setPID(
        AimingPIDS.toAngleRotKP, AimingPIDS.toAngleRotKI, AimingPIDS.toAngleRotKD);
    // TODO: might need this
    // headingReq.HeadingController.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    Rotation2d heading = mDrivetrain.getPose().getTranslation().minus(mTarget).getAngle();

    mDrivetrain.setControl(
        headingReq
            .withTargetDirection(heading)
            .withDeadband(DriveDeadband)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagic)
            .withVelocityX(controller.getLeftY())
            .withVelocityY(controller.getLeftX()));
  }
}
