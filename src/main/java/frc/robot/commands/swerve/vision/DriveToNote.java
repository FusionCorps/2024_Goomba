package frc.robot.commands.swerve.vision;

import static frc.robot.Constants.DrivetrainConstants.MaxSpeed;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// drive forward to note
public class DriveToNote extends Command{
    CommandSwerveDrivetrain mDrivetrain;
    double minTolerance = 18.0;
    double maxTolerance = 21.0;

    public DriveToNote(CommandSwerveDrivetrain drivetrain) {
        mDrivetrain = drivetrain;
        addRequirements(mDrivetrain);
    }

    @Override
    public void execute() {
        if (mDrivetrain.mCamera.hasTarget()) {
            double ty = mDrivetrain.mCamera.getTY();
            if (ty > minTolerance && ty < maxTolerance) {
                mDrivetrain.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(0.01*MaxSpeed));
            }
            else mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        }
    }
}
