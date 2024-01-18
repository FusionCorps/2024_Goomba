// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.swerve.manual.PointWheels;
import frc.robot.commands.swerve.manual.RunSwerveFC;
import frc.robot.commands.swerve.manual.SwerveBrake;
import frc.robot.commands.swerve.vision.AimAtTarget;
import frc.robot.commands.swerve.vision.RotateToAngle;
import frc.robot.commands.swerve.vision.StrafeToAprilTag;
import frc.robot.subsystems.*;

public class RobotContainer {
    public static CommandXboxController robotController = new CommandXboxController(0); // joystick
    public static CommandSwerveDrivetrain drivetrain = Constants.DrivetrainConstants.DriveTrain; // drivetrain
    Shooter shooter = new Shooter();
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

        // robotController.y().whileTrue(new AimAtTarget(drivetrain, 2.0, 0.5));
        robotController.y().whileTrue(drivetrain.aimAtTargetCommand(2.0, 0.5));
        // rotate to angle, strafe to april tag, and rumble controller 
        // robotController.x().whileTrue(
        //     new RotateToAngle(drivetrain, 180, 2.0, 0.75)
        // .andThen(Commands.print("rotation finished"))
        // .andThen(new StrafeToAprilTag(drivetrain, 2.0))
        // .andThen(Commands.print("strafe finished"))
        // .andThen(Commands.runOnce(() -> robotController.getHID().setRumble(RumbleType.kBothRumble, 0.2)))
        // .andThen(new WaitCommand(0.25))
        // .andThen(Commands.runOnce(() -> robotController.getHID().setRumble(RumbleType.kBothRumble, 0.0)))
        // );
        robotController.x().whileTrue(
            drivetrain.rotateToAngleCommand(180, 2.0, 0.75)
            .andThen(Commands.print("rotation finished"))
            .andThen(drivetrain.strafeToAprilTagCommand(2.0))
            .andThen(Commands.print("strafe finished"))
            .andThen(Commands.runOnce(() -> robotController.getHID().setRumble(RumbleType.kBothRumble, 0.2)))
            .andThen(new WaitCommand(0.25))
            .andThen(Commands.runOnce(() -> robotController.getHID().setRumble(RumbleType.kBothRumble, 0.0)))
        );

        robotController.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(
            new Pose2d(2, 0, new Rotation2d()))
        ));
        
        if (Utils.isSimulation())
            drivetrain.seedFieldRelative(new Pose2d(2, 0, new Rotation2d())); // in simulation, init robot position to (2, 0)
        else if (RobotBase.isReal())
            drivetrain.seedFieldRelative(); // in real life, set current heading to forward
        
        drivetrain.registerTelemetry(logger::telemeterize); // start telemetry

        robotController.leftBumper().whileTrue(new LaunchNote(shooter,.72));
    }

    public RobotContainer() {
        configureAuto();
        configureBindings();
    }

    // method that configures and initializes everything necessary for auton
    public void configureAuto() {
        autoChooser = AutoBuilder.buildAutoChooser("DoNothingAuto");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        NamedCommands.registerCommand("getAutoStartingPos",
                new InstantCommand(() -> System.out.println("Starting Pose: "
                        + PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.getSelected().getName()))));
        // testing the single path autons
        autoChooser.addOption("ScoreOne", drivetrain.singlePathToCommand("ScoreOne"));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
