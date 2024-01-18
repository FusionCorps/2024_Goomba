package frc.robot.commands.swerve.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


// rotates to a desired angle in place
public class RotateToAngle extends Command {
    CommandSwerveDrivetrain mDrivetrain;
    Timer timer = new Timer();
    double runTime; // how long in seconds to aim
    double desiredHeading = 0; // target angle to aim to, relative to forward in odometry

    SwerveRequest.FieldCentricFacingAngle rotReq; 


    public RotateToAngle(CommandSwerveDrivetrain drivetrain, double desiredHeadingDeg, double toleranceDeg, double runTime) {
        mDrivetrain = drivetrain;
        addRequirements(mDrivetrain);

        rotReq = new SwerveRequest.FieldCentricFacingAngle()
                .withTargetDirection(Rotation2d.fromDegrees(desiredHeadingDeg));
        // setup PID controller - notice that this controller uses radians units, and uses a continuous input range
        rotReq.HeadingController.setPID(
            PIDConstants.toAngleRotKP, 
            PIDConstants.toAngleRotKI,  
            PIDConstants.toAngleRotKD);
        rotReq.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        rotReq.HeadingController.setTolerance(Units.degreesToRadians(toleranceDeg));
        
        this.runTime = runTime;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
            // System.out.println("Error: " + rotReq.HeadingController.getPositionError()*180/Math.PI);
            mDrivetrain.setControl(rotReq);
    }

    @Override
    public boolean isFinished() {
        // stop aiming if we have been aiming for longer than runTime, or if we are at setpoint
        return timer.hasElapsed(runTime) || rotReq.HeadingController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // stop rotating
        mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        rotReq.HeadingController.reset();
    }
}
