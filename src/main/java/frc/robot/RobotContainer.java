// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.allianceColor;
import static frc.robot.Constants.allianceLocation;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.launcher.ShootSpeaker;
import frc.robot.commands.swerve.manual.PointWheels;
import frc.robot.commands.swerve.manual.RunSwerveFC;
import frc.robot.commands.swerve.manual.SwerveBrake;
import frc.robot.commands.swerve.vision.AimAtTarget;
import frc.robot.subsystems.*;

public class RobotContainer {
    public static CommandXboxController robotController = new CommandXboxController(0); // joystick
    public static Cameras mCameras = new Cameras();
    public static CommandSwerveDrivetrain drivetrain = Constants.DrivetrainConstants.DriveTrain; // drivetrain
    // Intake intake = new Intake();
    // Shooter shooter = new Shooter();
    private Telemetry logger = new Telemetry(DrivetrainConstants.MaxSpeed); // for logging data
    
    private SendableChooser<Command> autoChooser;

    
    /**
     * Configures the bindings for the robot's subsystems and commands.
     */
    private void configureBindings() {
        drivetrain.setDefaultCommand(new RunSwerveFC(drivetrain));
        robotController.a().whileTrue(new PointWheels(drivetrain));
        robotController.b().whileTrue(new SwerveBrake(drivetrain));

        // reset odometry to current position, and zero gyro yaw
        robotController.leftBumper().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldRelative(); 
            drivetrain.resetGyro();
        }));

        robotController.leftStick().whileTrue(drivetrain.aimAtTargetCommand(2.0, 0.5));
        robotController.y().whileTrue(new AimAtTarget(drivetrain, 2.0, 0.5));

        robotController.x().whileTrue(
            drivetrain.rotateToAngleCommand(180, 2.0, 0.75)
            // new RotateToAngle(drivetrain, 180, 2.0, 0.75)
            .andThen(Commands.print("rotation finished"))
            .andThen(drivetrain.strafeToAprilTagCommand(2.0))
            // .andThen(new StrafeToAprilTag(drivetrain, 2.0))
            .andThen(Commands.print("strafe finished"))
            .andThen(Commands.runOnce(() -> robotController.getHID().setRumble(RumbleType.kBothRumble, 0.2)))
            .andThen(new WaitCommand(0.1))
            .andThen(Commands.runOnce(() -> robotController.getHID().setRumble(RumbleType.kBothRumble, 0.0)))
        );

        robotController.rightBumper().toggleOnTrue(drivetrain.runSwerveFCwAim(
            robotController::getLeftY,
            robotController::getLeftX,
            2.0));
            

        // robotController.rightBumper().whileTrue(new ShootSpeaker(shooter,5000,3000));
        // robotController.leftStick().whileTrue(new RunIntake(intake));

    }

    public RobotContainer() {
        // get alliance color and location
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            Constants.allianceColor = alliance.get();
        } else System.err.println("Alliance not found");
        var location = DriverStation.getLocation();
        if (location.isPresent()) {
            allianceLocation = location.getAsInt();
        }

        configureAuto();
        configureBindings();

        // if (Utils.isSimulation()) {
        //     if (allianceColor.equals(DriverStation.Alliance.Blue)) {
        //         switch (allianceLocation) {
        //             case 1:
        //                 drivetrain.seedFieldRelative(new Pose2d(2, 2, new Rotation2d(0)));
        //                 break;
        //             case 2:
        //                 drivetrain.seedFieldRelative(new Pose2d(2, 4, new Rotation2d(0)));
        //                 break;
        //             case 3:
        //                 drivetrain.seedFieldRelative(new Pose2d(2, 7, new Rotation2d(0)));
        //                 break;
        //         }
        //     } else if (allianceColor.equals(DriverStation.Alliance.Red)) {
        //         switch (allianceLocation) {
        //             case 1:
        //                 drivetrain.seedFieldRelative(new Pose2d(16, 2, new Rotation2d(Math.PI)));
        //                 break;
        //             case 2:
        //                 drivetrain.seedFieldRelative(new Pose2d(16, 4, new Rotation2d(Math.PI)));
        //                 break;
        //             case 3:
        //                 drivetrain.seedFieldRelative(new Pose2d(16, 7, new Rotation2d(Math.PI)));
        //                 break;
        //         }
        //     }
        //     else {
        //         drivetrain.seedFieldRelative(new Pose2d()); // in simulation, set current heading to forward
        //     }
        // }
        if (Utils.isSimulation())
            drivetrain.seedFieldRelative(new Pose2d()); // set current heading to forward
        else drivetrain.seedFieldRelative(); // set current heading to forward

        drivetrain.registerTelemetry(logger::telemeterize); // start telemetry
    }

    // method that configures and initializes everything necessary for auton
    public void configureAuto() {
        autoChooser = AutoBuilder.buildAutoChooser("DoNothingAuto");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        NamedCommands.registerCommand("getAutoStartingPos",
                new InstantCommand(() -> System.out.println("Starting Pose: "
                        + PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.getSelected().getName()))));

        NamedCommands.registerCommand("ShootSpeaker", Commands.print("ShootSpeaker"));
        NamedCommands.registerCommand("ShootAmp", Commands.print("ShootAmp"));
        NamedCommands.registerCommand("RunIntake", Commands.print("RunIntake"));
        // NamedCommands.registerCommand("ShootSpeaker", new ShootSpeaker(shooter, 0.25));
        // NamedCommands.registerCommand("ShootAmp", new ShootSpeaker(shooter, 0.25));
        // NamedCommands.registerCommand("RunIntake", new RunIntake(intake));
        NamedCommands.registerCommand("AimAtTarget", Commands.print("AimAtTarget"));
        
        // testing the single path autons
        autoChooser.addOption("ScoreOne", drivetrain.singlePathToCommand("ScoreOne"));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
