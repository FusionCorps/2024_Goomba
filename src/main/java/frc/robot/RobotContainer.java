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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.RunIndex;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.launcher.AimShooterAngle;
import frc.robot.commands.launcher.ResetPivotAngle;
import frc.robot.commands.launcher.Shoot;
import frc.robot.commands.swerve.manual.RunSwerveFC;
import frc.robot.subsystems.*;

public class RobotContainer {
  public static CommandXboxController robotController = new CommandXboxController(0); // joystick
  public static CommandSwerveDrivetrain drivetrain =
      Constants.DrivetrainConstants.DriveTrain; // drivetrain

  // public TarsArm tarsArm = new TarsArm();
  Intake intake = new Intake();
  Shooter shooter = new Shooter();
  Index index = new Index();
  Pivot pivot = new Pivot();

  private Telemetry logger = new Telemetry(DrivetrainConstants.MaxSpeed); // for logging data

  private SendableChooser<Command> autoChooser;
  private SendableChooser<Integer> pipeLineChooser = new SendableChooser<>();

  /** Configures the bindings for the robot's subsystems and commands. */
  private void configureBindings() {
    drivetrain.setDefaultCommand(new RunSwerveFC(drivetrain));
    // reset odometry to current position, and zero gyro yaw
    robotController
        .b()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  drivetrain.seedFieldRelative();
                  drivetrain.resetGyro();
                }));

    // robotController.leftStick().whileTrue(drivetrain.aimAtTargetCommand(2.0, 0.5));
    // robotController.y().whileTrue(new AimAtTarget(drivetrain, 2.0, 0.5));

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
    // robotController.povLeft().onTrue(new SetBasePos(tarsArm,
    // Constants.TarsArmConstants.ARM_POS_1));
    // robotController.povRight().onTrue(new SetBasePos(tarsArm,
    // Constants.TarsArmConstants.ARM_POS_2));
    // robotController.povDown().onTrue(new SetBasePos(tarsArm,0));

    // robotController.rightBumper().whileTrue(new ShootSpeaker(shooter,5000,3000));
    // robotController.leftStick().whileTrue(new RunIntake(intake));

    // aim at speaker and shoot at speaker (TODO: add aiming at target)
    robotController.rightBumper().toggleOnTrue(new Shoot(shooter, 0.82, 0.67));
    // aim at amp and shoot at amp (TODO: add aiming at amp)
    robotController.leftBumper().whileTrue(new Shoot(shooter, 0.2, 0.2));

    // run index
    robotController.leftTrigger().whileTrue(new RunIndex(index, 0.24));

    // run outtake
    robotController.povDown().whileTrue(new RunIntake(intake, -0.75));

    // run intake
    robotController.rightTrigger().whileTrue(new RunIntake(intake, 0.75));

    // while the beam break is not broken, run the index
    new Trigger(index::beamBroken)
        .whileFalse(new RunIndex(index, 0.24))
        .onTrue(new RunIndex(index, 0));

    // reset pivot angle to current angle
    robotController.y().onTrue(new ResetPivotAngle(pivot));
    // robotController.b().onTrue(new SetShooterAngle(pivot, 0));

    // move shooter up or down
    robotController.povRight().whileTrue(new AimShooterAngle(pivot, .08));
    robotController.povLeft().whileTrue(new AimShooterAngle(pivot, -.08));
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

    // configureAuto();

    // set up pipeline chooser
    pipeLineChooser.setDefaultOption("AprilTag", 0);
    pipeLineChooser.addOption("Note", 1);
    // pipeLineChooser.onChange((num) -> {drivetrain.getCamera().setPipeline(num);
    // System.out.println("pipeline set to " + num);});
    SmartDashboard.putData("Pipeline Chooser", pipeLineChooser);

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
    //         drivetrain.seedFieldRelative(new Pose2d()); // in simulation, set current heading to
    // forward
    //     }
    // }
    // if (Utils.isSimulation())
    //     drivetrain.seedFieldRelative(new Pose2d()); // set current heading to forward
    // else drivetrain.seedFieldRelative(); // set current heading to forward

    drivetrain.registerTelemetry(logger::telemeterize); // start telemetry
  }

  // method that configures and initializes everything necessary for auton
  public void configureAuto() {
    NamedCommands.registerCommand("ShootSpeaker", Commands.print("ShootSpeaker"));
    NamedCommands.registerCommand("ShootAmp", Commands.print("ShootAmp"));
    NamedCommands.registerCommand("RunIntake", Commands.print("RunIntake"));
    NamedCommands.registerCommand("AimAtTarget", Commands.print("AimAtTarget"));
    NamedCommands.registerCommand(
        "getAutoStartingPos",
        new InstantCommand(
            () ->
                System.out.println(
                    "Starting Pose: "
                        + PathPlannerAuto.getStaringPoseFromAutoFile(
                            autoChooser.getSelected().getName()))));

    // testing the single path autons
    // autoChooser.addOption("ScoreOne", drivetrain.singlePathToCommand("ScoreOne"));

    System.out.println(AutoBuilder.getAllAutoNames());
    // if this throws an error, make sure all autos are complete
    // can verify what paths/autos are on rio: ftp://roboRIO-6672-frc.local
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
