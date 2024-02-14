// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DrivetrainConstants.MaxSpeed;
import static frc.robot.Constants.IntakeConstants.INTAKE_RUN_PCT;
import static frc.robot.Constants.ShooterConstants.AMP_LEFT_SPEED;
import static frc.robot.Constants.ShooterConstants.AMP_RIGHT_SPEED;
import static frc.robot.Constants.ShooterConstants.SPK_LEFT_SPEED;
import static frc.robot.Constants.ShooterConstants.SPK_RIGHT_SPEED;
import static frc.robot.Constants.allianceLocation;
import static frc.robot.Constants.driverTab;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.StageAlignment;
import frc.robot.commands.RunIndex;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.launcher.ResetPivotAngle;
import frc.robot.commands.launcher.SetPivotPct;
import frc.robot.commands.launcher.SetPivotPos;
import frc.robot.commands.launcher.Shoot;
import frc.robot.commands.swerve.manual.RunSwerveFC;
import frc.robot.commands.swerve.vision.AimAtTarget;
import frc.robot.commands.swerve.vision.DriveToNote;
import frc.robot.commands.swerve.vision.RotateToAngle;
import frc.robot.commands.swerve.vision.StrafeToAprilTag;
import frc.robot.subsystems.*;

public class RobotContainer {
  public static CommandXboxController robotController = new CommandXboxController(0);
  public static Drivetrain drivetrain = Constants.DrivetrainConstants.DriveTrain;

  Intake intake = new Intake();
  Shooter shooter = new Shooter();
  Index index = new Index();
  Pivot pivot = new Pivot();

  private SendableChooser<Command> autoChooser;
  private SendableChooser<Integer> pipeLineChooser = new SendableChooser<>();

  /** Configures the bindings for the robot's subsystems and commands. */
  private void configureBindings() {
    // run field centric swerve drive
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

    // aim at speaker and shoot at speaker (TODO: add aiming at target)
    robotController.rightBumper().toggleOnTrue(new Shoot(shooter, 0.82, 0.67));
    // TODO: shooter velocity PID control
    // robotController.rightBumper().whileTrue(new ShootSpeaker(shooter,5000,3000));

    // aim at amp and shoot at amp 
    // robotController.leftBumper().onTrue(new SequentialCommandGroup(new RotateToAngle(drivetrain, 90, MaxSpeed, 0.3), 
    //                                                                 new ParallelCommandGroup(new StrafeToAprilTag(drivetrain, StageAlignment.toleranceDeg)), 
    //                                                                                           new SetPivotPos(pivot, PivotConstants.PIVOT_AMP_POS)));

    // run index
    robotController.leftTrigger().whileTrue(new RunIndex(index, IndexConstants.INDEX_PCT));

    // run intake
    robotController.rightTrigger().whileTrue(new RunIntake(intake, INTAKE_RUN_PCT));

    // run outtake
    robotController.povDown().whileTrue(new RunIntake(intake, -INTAKE_RUN_PCT));
    //robotController.povLeft().whileTrue(new RunIntake(intake, -INTAKE_RUN_PCT));

    // while the beam break sensor is not broken, run the index
    new Trigger(index::beamBroken)
        .whileFalse(new RunIndex(index, IndexConstants.INDEX_PCT))
        .onTrue(new RunIndex(index, 0));

    // zero the pivot angle at current angle
    robotController.y().onTrue(new ResetPivotAngle(pivot));

    // disables the robot
    robotController.x().onTrue(Commands.run(() -> CommandScheduler.getInstance().disable()));

    // move shooter up or down
    robotController.povRight().whileTrue(new SetPivotPct(pivot, .08));
    robotController.povLeft().whileTrue(new SetPivotPct(pivot, -.08));

    // Move climb to upright position
    //robotController.povUp().onTrue(new SetPivotPos(pivot, PivotConstants.PIVOT_CLIMB_UP_POS));
    //robotController.povDown().whileTrue(new SetPivotPct(pivot, -0.4));

    // robotController.leftStick().whileTrue(new AimAtTarget(drivetrain, 2.0, 0.5)); // rotate in
    // place to aim at target

    // rotate to perpendicular with wall, strafe to center on april tag, and rumble
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

    setupAutoChooser();
    setupPipelineChooser();

    configureBindings();
  }

  private void setupPipelineChooser() {
    pipeLineChooser.setDefaultOption("AprilTag 3D", 0);
    pipeLineChooser.addOption("AprilTag Basic", 1);
    pipeLineChooser.addOption("Note", 2);
    pipeLineChooser.onChange(
        (num) -> {
          drivetrain.getCamera().setPipeline(num);
          System.out.println("pipeline set to " + num);
        });
    driverTab.add("Pipeline Chooser", pipeLineChooser).withSize(2, 1).withPosition(4, 3);
  }

  // method that configures and initializes everything necessary for auton
  private void setupAutoChooser() {
    NamedCommands.registerCommand("ShootSpeaker", new Shoot(shooter, SPK_LEFT_SPEED, SPK_RIGHT_SPEED));
    NamedCommands.registerCommand("ShootAmp", new Shoot(shooter, AMP_LEFT_SPEED, AMP_RIGHT_SPEED));
    NamedCommands.registerCommand("RunIntake", new RunIntake(intake, INTAKE_RUN_PCT));
    NamedCommands.registerCommand("AimAtTarget", new AimAtTarget(drivetrain, StageAlignment.toleranceDeg, StageAlignment.runTime));
    NamedCommands.registerCommand(
        "getAutoStartingPos",
        Commands.runOnce(
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

    driverTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(4, 2);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
