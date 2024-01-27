package frc.robot.commands.swerve.manual;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * This command allows the driver to strafe the robot,
 * It locks the robot's angle and disables forward/backward movement.
 */
public class StrafeOnly extends Command {
    private CommandSwerveDrivetrain mDrivetrain;
    private CommandXboxController controller = RobotContainer.robotController;
    private Rotation2d targetDirection;

    public StrafeOnly(CommandSwerveDrivetrain drivetrain) {
        mDrivetrain = drivetrain;
        addRequirements(mDrivetrain);
    }

    @Override
    public void initialize() {
        targetDirection = mDrivetrain.getPose().getRotation();
    }

    @Override
    public void execute() {
        mDrivetrain.setControl(new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(0.05)
                .withVelocityX(0.0)
                .withVelocityY(-controller.getLeftX())
                .withTargetDirection(targetDirection));
    }    
}
