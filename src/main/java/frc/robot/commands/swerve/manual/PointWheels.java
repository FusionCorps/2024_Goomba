package frc.robot.commands.swerve.manual;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Point wheels in certain direction, according to controller input
 */
public class PointWheels extends Command {
    private CommandSwerveDrivetrain mDrivetrain;
    private CommandXboxController controller = RobotContainer.robotController;

    public PointWheels(CommandSwerveDrivetrain drivetrain) {
        mDrivetrain = drivetrain;
        addRequirements(mDrivetrain);
    }

    @Override
    public void execute() {
        mDrivetrain.setControl(new SwerveRequest.PointWheelsAt()
                .withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX())));
    }
}
