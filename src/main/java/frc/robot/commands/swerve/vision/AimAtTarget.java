package frc.robot.commands.swerve.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


// aims at a target in place
public class AimAtTarget extends Command {
    private CommandSwerveDrivetrain mDrivetrain;
    PIDController pid = new PIDController(
        PIDConstants.toTargetRotKP, 
        PIDConstants.toTargetRotKI,  
        PIDConstants.toTargetRotKD);
    Timer timer = new Timer();
    private double runTime;
        
    double tx = 0.0;

    public AimAtTarget(CommandSwerveDrivetrain drivetrain, double toleranceDeg, double runTime) {
        timer.restart();
        pid.setTolerance(toleranceDeg);
        pid.setSetpoint(0.0); // goal is to have tx be 0 (centered on target)
        this.runTime = runTime;
        mDrivetrain = drivetrain;

        addRequirements(mDrivetrain);
    }

    @Override
    public void execute() {
        // if target detected, rotate to target
        if (mDrivetrain.hasTarget()) {
            double tx = mDrivetrain.getTX();
            // get pid output of normalized tx (-1 to 1) and scale by max angular rate
            SwerveRequest req = new SwerveRequest.FieldCentric().withRotationalRate(
                    pid.calculate(tx / Constants.LIMELIGHT_TX_RANGE_DEG) * DrivetrainConstants.MaxAngularRate);
            mDrivetrain.setControl(req);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(runTime);
    }

    @Override
    public void end(boolean interrupted) {
        // stop rotating
        mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        pid.close();
    }
}
