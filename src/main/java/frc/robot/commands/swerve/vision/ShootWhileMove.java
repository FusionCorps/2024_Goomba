// package frc.robot.commands.swerve.vision;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Shooter;

// public class ShootWhileMove extends Command {
//     private final Shooter mShooter;
//     private final CommandSwerveDrivetrain mDrivetrain;
//     private final CommandXboxController robotController = RobotContainer.robotController;

//     public ShootWhileMove(Shooter shooter, CommandSwerveDrivetrain drive){
//         mShooter = shooter;
//         mDrivetrain = drive;
//         addRequirements(mShooter, mDrivetrain);
//     }

//     @Override
//     public void execute() {
//         // lateral movement controlled manually, aiming and shooting automatic
//         SwerveRequest req;

//         if (mDrivetrain.getCamera().hasTarget() && mDrivetrain.getCamera().getPipeline() == 0) {
//             ChassisSpeeds drivetrainSpeeds = mDrivetrain.getFieldRelativeSpeeds();

//             // FieldRelativeAccel robotAccel = m_drive.getFieldRelativeAccel();

//             // Translation2d target = GoalConstants.kGoalLocation;

//             // Translation2d robotToGoal = target.minus(mDrivetrain.getPose().getTranslation());
//             // double dist = robotToGoal.getDistance(new Translation2d())*39.37;

//             // get estimated translation of target
//             Pose3d targetPose = mDrivetrain.getCamera().getPrimaryAprilTagPose();

//             Translation2d movingGoalLocation = targetPose.getTranslation().toTranslation2d();

//             Translation2d toMovingGoal =
// movingGoalLocation.minus(mDrivetrain.getPose().getTranslation());

//             double newDist = toMovingGoal.getDistance(new Translation2d())*39.37;

//             mShooter.shoot(5000, 3000);
//             // m_turret.aimAtGoal(m_drive.getPose(), movingGoalLocation, false);
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         mShooter.shoot(0, 0);
//     }
// }
