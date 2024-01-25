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
    double tolerance = 18.0;

    public DriveToNote(CommandSwerveDrivetrain drivetrain) {
        mDrivetrain = drivetrain;
        addRequirements(mDrivetrain);
    }

    @Override
    public void execute() {
        System.out.println("running DriveToNote");
        if (mDrivetrain.mCamera.hasTarget()) {
            double ty = mDrivetrain.mCamera.getTY();
            if (ty < tolerance) {
                SwerveRequest req = new SwerveRequest.RobotCentric()
                    .withVelocityX(-0.1*MaxSpeed);
                mDrivetrain.setControl(req);
            }
            else mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        }
    }
}
