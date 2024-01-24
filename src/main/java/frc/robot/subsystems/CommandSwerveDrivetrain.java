package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.DrivetrainConstants.MaxAngularRate;

import java.util.function.DoubleSupplier;

/**
 * This class represents a command-based swerve drivetrain subsystem.
 * It extends the SwerveDrivetrain class and implements the Subsystem interface.
 * It provides methods for controlling the swerve drivetrain, updating odometry,
 * and executing various commands related to the drivetrain.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    // current distance to target
    public Cameras mCamera;

    public CommandSwerveDrivetrain(
        Cameras camera,
        SwerveDrivetrainConstants driveTrainConstants,
        double OdometryUpdateFrequency,
        SwerveModuleConstants... modules
        ) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        mCamera = camera;
        configPathPlanner();

        // vision measurement std filter
        setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); 

        // VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(5)); // state filter, doesn't seem to be a way to use this in the constructor
    }

    @Override
    public void periodic() { 
        // updates the odometry from aprilTag data
        updateOdometryFromAprilTags(1.0);
    }

    @Override
    public void simulationPeriodic() {
        /* Assume */
        updateSimState(0.02, 12);
    }

    /**
     * Configures the PathPlanner's AutoBuilder.
     */
    private void configPathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::seedFieldRelative,
            this::getRobotRelativeSpeeds,
            this::driveRobotRelative,
            new HolonomicPathFollowerConfig(
                new PIDConstants(8, 0, 0.25), // translational
                new PIDConstants(8, 0, 0.25), // rotational
                DrivetrainConstants.MaxSpeed,
                DrivetrainConstants.DRIVEBASE_RADIUS,
                // new ReplanningConfig()),
                new ReplanningConfig(false, false)),
            this::isAllianceRed,
            this);
    }

    /**
     * @return true if the robot is on the red alliance, false if on the blue alliance or if the alliance color is unknown
     */
    private boolean isAllianceRed() {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        if (Constants.allianceColor != null) {
            return Constants.allianceColor == DriverStation.Alliance.Red;
        }
        return false;
    }    

    /**
     * Updates the odometry from the latest april tag data.
     * If the robot is real and the limelight detects a target,
     * the robot's position and orientation will be updated.
     * @param maxDist the maximum distance to the target in meters required for odometry to be updated, recommended <= 1.0
     */
    private void updateOdometryFromAprilTags(double maxDist) {
        if (RobotBase.isReal() && mCamera.hasTarget() && mCamera.distToTargetMeters < maxDist) {
                Pose2d pose = LimelightHelpers.getBotPose2d(Constants.LIMELIGHT_NAME);
                // System.out.println("Pose update: " + pose);
                // double timestamp = Timer.getFPGATimestamp();
                double timestamp = Timer.getFPGATimestamp()
                - LimelightHelpers.getLatency_Capture(Constants.LIMELIGHT_NAME) / 1000.0
                - LimelightHelpers.getLatency_Pipeline(Constants.LIMELIGHT_NAME) / 1000.0;
                addVisionMeasurement(pose, timestamp);
        }
    }

    /*
     * Returns a command that runs the specified PathPlanner path.
     * The robot will first reset its odometry to the initial pose of the path,
     * then run the path.
     */
    public Command singlePathToCommand(String pathName) {
        // the path
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        // the initial pose of the path
        Pose2d initPose = path.getPreviewStartingHolonomicPose();

        // reset the odometry to the initial pose of the path, then run the path
        return runOnce(() -> seedFieldRelative(initPose))
            .andThen(AutoBuilder.followPath(path));
    }

    /**
     * Drives the robot relative to its current orientation using the specified speeds.
     * @param speeds The desired chassis speeds in meters per second.
     */
    private void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveRequest request = new SwerveRequest.RobotCentric()
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond);
        setControl(request);
    }

    /**
     * @return The current pose of the robot.
     */
    private Pose2d getPose() {
        return getState().Pose;
    }

    /**
     * @return The current robot-relative chassis speeds of the robot.
     */
    private ChassisSpeeds getRobotRelativeSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    /**
     * Zeroes the gyro yaw.
     */
    public void resetGyro() {
        m_pigeon2.reset();
    }

    /**
     * Rotates to a target in place using the limelight.
     * @param toleranceDeg the tolerance in degrees
     * @param runTime the maximum time to run the command
     * @return the command
     */
    public Command aimAtTargetCommand(double toleranceDeg, double runTime) {
        PIDController pid = new PIDController(
            Constants.PIDConstants.toTargetRotKP, 
            Constants.PIDConstants.toTargetRotKI,  
            Constants.PIDConstants.toTargetRotKD);
        pid.setTolerance(toleranceDeg);
        pid.setSetpoint(0.0); // goal is to have tx be 0 (centered on target)
        return run(() -> {
                // if target detected, rotate to target
                if (mCamera.hasTarget()) {
                    double tx = mCamera.getTX();
                    // get pid output of normalized tx (-1 to 1) and scale by max angular rate
                    SwerveRequest req = new SwerveRequest.FieldCentric().withRotationalRate(
                            pid.calculate(tx / Constants.LIMELIGHT_TX_RANGE_DEG) * MaxAngularRate);
                    setControl(req);
                }
            })
        // .until(pid::atSetpoint)
        .withTimeout(runTime)
        .finallyDo(() -> {
            // stop rotating
            setControl(new SwerveRequest.SwerveDriveBrake());
            pid.close();
        });
    }

    /**
     * Rotates to an angle in place.
     * @param desiredHeadingDeg the desired heading in degrees
     * @param toleranceDeg the tolerance in degrees
     * @param runTime the maximum time to run the command
     * @return the command
     */
    public Command rotateToAngleCommand(double desiredHeadingDeg, double toleranceDeg, double runTime) {
        SwerveRequest.FieldCentricFacingAngle rotReq = new SwerveRequest.FieldCentricFacingAngle()
                .withTargetDirection(Rotation2d.fromDegrees(desiredHeadingDeg));
        // setup PID controller - notice that this controller uses radians units, and uses a continuous input range
        rotReq.HeadingController.setPID(
            Constants.PIDConstants.toAngleRotKP, 
            Constants.PIDConstants.toAngleRotKI,  
            Constants.PIDConstants.toAngleRotKD);
        rotReq.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        rotReq.HeadingController.setTolerance(Units.degreesToRadians(toleranceDeg));
        return run(() -> {
            System.out.println("Error: " + rotReq.HeadingController.getPositionError()*180/Math.PI);
            setControl(rotReq);
        })
        .until(rotReq.HeadingController::atSetpoint)
        .withTimeout(runTime)
        .finallyDo(() -> {
            // stop rotating
            setControl(new SwerveRequest.SwerveDriveBrake());
            rotReq.HeadingController.reset();
        });
    }

    /**
     * Strafes to an april tag using the limelight.
     * @param toleranceDeg the tolerance in degrees
     * @return the command
     */
    public Command strafeToAprilTagCommand(double toleranceDeg) {
        PIDController strPID = new PIDController(
            Constants.PIDConstants.strKP, 
            Constants.PIDConstants.strKI,  
            Constants.PIDConstants.strKD);
        strPID.setTolerance(toleranceDeg); // strafe to within tolerance degrees of target
        strPID.setSetpoint(0.0); // goal is to have tx be 0 (centered on target)
        return run(() -> {
            // if target detected, strafe toward it
            if (mCamera.hasTarget()) {
                // System.out.println("Strafe error: " + strPID.getPositionError());
                double tx = mCamera.getTX();
                // normalize tx to be between -1 and 1, then scale by max angular rate
                // set strafe velocity to velY * scaling factor, negate for correct direction
                SwerveRequest req = new SwerveRequest.FieldCentric().withVelocityY(
                        strPID.calculate(tx / Constants.LIMELIGHT_TX_RANGE_DEG) 
                        * DrivetrainConstants.MaxSpeed)
                        .withVelocityX(0.0);
                setControl(req);
            }
        })
        .until(() -> Math.abs(mCamera.getTX()) <= toleranceDeg)
        .andThen(runOnce(() -> {
            // stop strafing
            setControl(new SwerveRequest.SwerveDriveBrake());
            strPID.close();
        }));
    }

    /**
     * Points the swerve wheels in the specified direction.
     * @param x the x component of the direction vector
     * @param y the y component of the direction vector
     * @return the command
     */
    public Command pointWheelsCommand(DoubleSupplier x, DoubleSupplier y) {
        return run(() -> setControl(new SwerveRequest.PointWheelsAt()
        .withModuleDirection(new Rotation2d(-x.getAsDouble(), -y.getAsDouble()))));
    }

    /**
     * Sets the swerve drivetrain to brake mode.
     * @return the command
     */
    public Command brakeCommand() {
        return run(() -> setControl(new SwerveRequest.SwerveDriveBrake()));
    }

    

    /**
     * Runs the swerve drive command with field-centric control.
     * @param x the x-axis (forward) value for velocity control
     * @param y the y-axis (strafe) value for velocity control
     * @param rot the rotational rate (counter-clockwise) value for velocity control
     * @return the command to be executed
     */
    public Command runSwerveFCCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        double deadband = 0.05;
        return run(() -> setControl(new SwerveRequest.FieldCentric()
                .withDeadband(deadband)
                .withVelocityX(-x.getAsDouble() * DrivetrainConstants.MaxSpeed)
                .withVelocityY(-y.getAsDouble() * DrivetrainConstants.MaxSpeed)
                .withRotationalRate(-rot.getAsDouble() * MaxAngularRate)));
    }

    /**
     * Runs the swerve drive command with robot-centric control.
     * @param x the x-axis (forward) value for velocity control
     * @param y the y-axis (strafe) value for velocity control
     * @param rot the rotational rate (counter-clockwise) value for velocity control
     * @return the command to be executed
     */
    public Command runSwerveRCCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        double deadband = 0.05;
        return run(() -> setControl(new SwerveRequest.RobotCentric()
                .withDeadband(deadband)
                .withVelocityX(-x.getAsDouble() * DrivetrainConstants.MaxSpeed)
                .withVelocityY(-y.getAsDouble() * DrivetrainConstants.MaxSpeed)
                .withRotationalRate(-rot.getAsDouble() * MaxAngularRate)));
    }

    public Command runSwerveFCwAim(DoubleSupplier x, DoubleSupplier y, double toleranceDeg) {
        PIDController pid = new PIDController(
            Constants.PIDConstants.toTargetRotKP, 
            Constants.PIDConstants.toTargetRotKI,  
            Constants.PIDConstants.toTargetRotKD);
        double deadband = 0.05;
        double movementDamp = 0.3; // percentage
        pid.setTolerance(toleranceDeg);
        pid.setSetpoint(0.0); // goal is to have tx be 0 (centered on target)
        return run(() -> {
            if (mCamera.hasTarget()) {
                double tx = mCamera.getTX();
                // get pid output of normalized tx (-1 to 1) and scale by max angular rate
                setControl(new SwerveRequest.FieldCentric()
                    .withDeadband(deadband)
                    .withVelocityX(movementDamp * -x.getAsDouble() * DrivetrainConstants.MaxSpeed)
                    .withVelocityY(movementDamp * -y.getAsDouble() * DrivetrainConstants.MaxSpeed)
                    .withRotationalRate(
                        pid.calculate(tx / Constants.LIMELIGHT_TX_RANGE_DEG) * MaxAngularRate
                    ));
            }})
            .until(() -> !mCamera.hasTarget())
            .finallyDo(() -> {
                // stop rotating
                setControl(new SwerveRequest.SwerveDriveBrake());
                pid.close();
            }
        );
    }
}
