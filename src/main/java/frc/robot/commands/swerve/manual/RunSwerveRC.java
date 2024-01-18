package frc.robot.commands.swerve.manual;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RunSwerveRC extends Command {
    CommandSwerveDrivetrain mDrivetrain;
    CommandXboxController controller = RobotContainer.robotController;
    private boolean presetInputs = false;

    double velX = 0, velY = 0, rot = 0;
    double deadband = 0.05;

    SwerveRequest request = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(deadband);

    public RunSwerveRC(CommandSwerveDrivetrain drivetrain) {
        mDrivetrain = drivetrain;
        addRequirements(mDrivetrain);
    }

    public RunSwerveRC(CommandSwerveDrivetrain drivetrain, double fwd, double strafe, double rotation) {
        mDrivetrain = drivetrain;
        addRequirements(mDrivetrain);
        this.velX = fwd;
        this.velY = strafe;
        this.rot = rotation;
        presetInputs = true;
    }

    @Override
    /*
     * This method is called periodically while the command is scheduled.
     * Robot-centric Movement, open-loop movement
     * Drive forward with negative Y (left joystick)
     * Drive left with negative X (left joystick)
     * Rotate counterclockwise with negative X (right joystick)
     */
    public void execute() {
        // depending on presetInputs, use either controller input or preset values
        velX = (presetInputs ? velX : -controller.getLeftY()) * DrivetrainConstants.MaxSpeed;
        velY = (presetInputs ? velY : -controller.getLeftX()) * DrivetrainConstants.MaxSpeed;
        rot = (presetInputs ? rot : -controller.getRightX()) * DrivetrainConstants.MaxAngularRate;

        SwerveRequest request = new SwerveRequest.RobotCentric()
                .withDeadband(deadband)
                .withVelocityX(velX)
                .withVelocityY(velY)
                .withRotationalRate(rot);
        mDrivetrain.setControl(request);
    }
}
