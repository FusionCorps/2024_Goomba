package frc.robot.commands.swerve.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// horizontally strafes to align with a target
public class StrafeToAprilTag extends Command {
    private final CommandSwerveDrivetrain mDrivetrain;
    double tx;
    PIDController strPID = new PIDController(
            PIDConstants.strKP, 
            PIDConstants.strKI,  
            PIDConstants.strKD);
    double toleranceDeg;
        
    public StrafeToAprilTag(CommandSwerveDrivetrain drivetrain, double toleranceDeg) {
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
        if (mDrivetrain.mCamera.hasTarget()) {
            System.out.println("Strafe error: " + strPID.getPositionError());
            double tx = mDrivetrain.mCamera.getTX();
            // normalize tx to be between -1 and 1, then scale by max angular rate
            // set strafe velocity to velY * scaling factor
            SwerveRequest req = new SwerveRequest.FieldCentric().withVelocityY(
                    strPID.calculate(tx / Constants.LIMELIGHT_TX_RANGE_DEG) 
                    * DrivetrainConstants.MaxAngularRate)
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

