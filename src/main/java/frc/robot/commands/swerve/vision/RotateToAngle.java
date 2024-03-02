package frc.robot.commands.swerve.vision;

import static frc.robot.Constants.allianceColor;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimingPIDS;
import frc.robot.subsystems.Drivetrain;

// rotates to a desired angle in place - angle is measured counterclockwise
public class RotateToAngle extends Command {
  private Drivetrain mDrivetrain;

  SwerveRequest.FieldCentricFacingAngle rotReq;

  public RotateToAngle(Drivetrain drivetrain, double desiredHeadingDeg, double toleranceDeg) {
    mDrivetrain = drivetrain;
    addRequirements(mDrivetrain);

    rotReq =
        new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(
                Rotation2d.fromDegrees(
                    allianceColor == Alliance.Blue ? desiredHeadingDeg : -desiredHeadingDeg));
    // setup PID controller - notice that this controller uses radians units, and uses a continuous
    // input range
    rotReq.HeadingController.setPID(
        AimingPIDS.toAngleRotKP, AimingPIDS.toAngleRotKI, AimingPIDS.toAngleRotKD);
    rotReq.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    rotReq.HeadingController.setTolerance(Units.degreesToRadians(toleranceDeg));
  }

  @Override
  public void execute() {
    // System.out.println("Error: " + rotReq.HeadingController.getPositionError()*180/Math.PI);
    mDrivetrain.setControl(rotReq);
  }

  @Override
  public boolean isFinished() {
    // stop aiming if we have been aiming for longer than runTime, or if we are at setpoint
    return rotReq.HeadingController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    // stop rotating
    mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    rotReq.HeadingController.reset();
  }
}
