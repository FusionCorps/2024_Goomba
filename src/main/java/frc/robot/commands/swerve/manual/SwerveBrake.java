package frc.robot.commands.swerve.manual;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveBrake extends Command {
    CommandSwerveDrivetrain mDrivetrain;

    public SwerveBrake(CommandSwerveDrivetrain drivetrain) {
        mDrivetrain = drivetrain;
        addRequirements(mDrivetrain);
    }

    @Override
    /*
     * This method is called periodically while the command is scheduled.
     * Lock wheels in X shape
     */
    public void execute() {
        mDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}
