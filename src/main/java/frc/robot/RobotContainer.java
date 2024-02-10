// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.allianceLocation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.swerve.manual.RunSwerveFC;
import frc.robot.commands.swerve.vision.AimAtTarget;
import frc.robot.commands.swerve.vision.DriveToNote;
import frc.robot.subsystems.*;

public class RobotContainer {
  public static CommandXboxController robotController = new CommandXboxController(0); // joystick
  public static CommandSwerveDrivetrain drivetrain =
      Constants.DrivetrainConstants.DriveTrain; // drivetrain

  private Telemetry logger = new Telemetry(DrivetrainConstants.MaxSpeed); // for logging data

  private SendableChooser<Command> autoChooser;
  private SendableChooser<Integer> pipeLineChooser = new SendableChooser<>();

  /** Configures the bindings for the robot's subsystems and commands. */
  private void configureBindings() {
    drivetrain.setDefaultCommand(new RunSwerveFC(drivetrain));
    // robotController.a().whileTrue(new PointWheels(drivetrain));
    // robotController.leftBumper().toggleOnTrue(new SwerveBrake(drivetrain));

    // // reset odometry to current position, and zero gyro yaw
    robotController
        .b()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  drivetrain.seedFieldRelative();
                  drivetrain.resetGyro();
                }));

    robotController.y().whileTrue(new AimAtTarget(drivetrain, 2.0, 0.5));
    robotController.a().whileTrue(new DriveToNote(drivetrain));
    // robotController
    //     .x()
    //     .whileTrue(
    //         drivetrain
    //             .rotateToAngleCommand(180, 2.0, 0.75)
    //             // new RotateToAngle(drivetrain, 180, 2.0, 0.75)
    //             .andThen(Commands.print("rotation finished"))
    //             .andThen(drivetrain.strafeToAprilTagCommand(2.0))
    //             // .andThen(new StrafeToAprilTag(drivetrain, 2.0))
    //             .andThen(Commands.print("strafe finished"))
    //             .andThen(
    //                 Commands.runOnce(
    //                     () -> robotController.getHID().setRumble(RumbleType.kBothRumble, 0.2)))
    //             .andThen(new WaitCommand(0.1))
    //             .andThen(
    //                 Commands.runOnce(
    //                     () -> robotController.getHID().setRumble(RumbleType.kBothRumble, 0.0))));

    // // TODO: run with AimShooterAngle
    // robotController
    //     .rightBumper()
    //     .toggleOnTrue(
    //         drivetrain.runSwerveFCwAim(robotController::getLeftY, robotController::getLeftX,
    // 2.0));

    // robotController
    //     .a()
    //     .whileTrue(new AimAtTarget(drivetrain, 3.0, 0.25).andThen(new DriveToNote(drivetrain)));
    // robotController
    //     .povUp()
    //     .whileTrue(drivetrain.runSwerveFCCommand(() -> -0.01 * MaxSpeed, () -> 0, () -> 0));
    // robotController
    //     .povDown()
    //     .whileTrue(drivetrain.runSwerveFCCommand(() -> 0.01 * MaxSpeed, () -> 0, () -> 0));
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
    } else System.err.println("Location not found");

    configureAuto();

    // set up pipeline chooser
    pipeLineChooser.setDefaultOption("AprilTag", 0);
    pipeLineChooser.addOption("Note", 1);
    // pipeLineChooser.onChange((num) -> drivetrain.getCamera().setPipeline(num));
    // SmartDashboard.putData("Pipeline Chooser", pipeLineChooser);

    configureBindings();

    // drivetrain.registerTelemetry(logger::telemeterize); // start telemetry
  }

  // method that configures and initializes everything necessary for auton
  public void configureAuto() {
    NamedCommands.registerCommand("Run Flywheel", Commands.print("Run Flywheel"));
    NamedCommands.registerCommand("ShootSpeaker", Commands.print("ShootSpeaker"));
    NamedCommands.registerCommand("ShootAmp", Commands.print("ShootAmp"));
    NamedCommands.registerCommand("RunIntake", Commands.print("RunIntake"));
    NamedCommands.registerCommand("AimAtTarget", Commands.print("AimAtTarget"));
    NamedCommands.registerCommand(
        "getAutoStartingPos",
        Commands.print(
            "Starting Pose: "
                + PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.getSelected().getName())));

    // testing the single path autons
    // autoChooser.addOption("ScoreOne", drivetrain.singlePathToCommand("ScoreOne"));

    // verify what paths/autos are on rio: ftp://roboRIO-6672-frc.local
    autoChooser = AutoBuilder.buildAutoChooser();

    // SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
