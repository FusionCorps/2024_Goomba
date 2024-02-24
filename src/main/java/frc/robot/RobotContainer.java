// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.IndexConstants.INDEX_PCT;
import static frc.robot.Constants.IntakeConstants.INTAKE_RUN_PCT;
import static frc.robot.Constants.ShooterConstants.AMP_LEFT_SPEED;
import static frc.robot.Constants.ShooterConstants.AMP_RIGHT_SPEED;
import static frc.robot.Constants.ShooterConstants.SPK_LEFT_RPM;
import static frc.robot.Constants.ShooterConstants.SPK_RIGHT_RPM;
import static frc.robot.Constants.allianceLocation;
import static frc.robot.Constants.driverTab;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.LimelightConstants.PIPELINE;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StageAlignment;
import frc.robot.commands.index.RunIndex;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.pivot.AutoPivotAim;
import frc.robot.commands.pivot.SetPivotPct;
import frc.robot.commands.pivot.SetPivotPos;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.swerve.vision.AimAtTarget;
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

    // Run Index default

    index.setDefaultCommand(new RunIndex(index, INDEX_PCT));

    // run field centric swerve drive
    // drivetrain.setDefaultCommand(
    //     DRIVE_MODE_CURRENT == DRIVE_MODE.FIELD_CENTRIC
    //         ? new RunSwerveFC(drivetrain)
    //         : new RunSwerveRC(drivetrain));

    // reset odometry to current position, and zero gyro yaw
    robotController
        .b()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  drivetrain.seedFieldRelative();
                  drivetrain.resetGyro();
                }));

    robotController.back().onTrue(pivot.runOnce(() -> pivot.syncPosition()));
    // aim at speaker and shoot at speaker (TODO: add aiming at target)
    // robotController
    //     .rightBumper()
    //     .toggleOnTrue(new Shoot(shooter, index, 0.82, 0.67).alongWith(new SetPivotPos(pivot,
    // 24.51)));

    // robotController
    //     .rightBumper()
    //     .toggleOnTrue(new AimAtTarget(drivetrain, StageAlignment.toleranceDeg));
    // robotController.rightBumper().toggleOnTrue(new AutoPivotAim(drivetrain, pivot));
    // robotController.rightBumper().toggleOnTrue(new Shoot(shooter, index, 0.82, 0.67));
    robotController
        .rightBumper()
        .toggleOnTrue(
            new Shoot(shooter, index, ShooterConstants.SPK_LEFT_RPM, ShooterConstants.SPK_RIGHT_RPM)
            .alongWith(new AimAtTarget(drivetrain, StageAlignment.toleranceDeg)
            .alongWith(new AutoPivotAim(pivot, drivetrain.getCamera()).repeatedly())));
    robotController.y().onTrue(new SetPivotPos(pivot, 24.51));
    // robotController.rightBumper().toggleOnTrue(new ParallelCommandGroup(new Shoot(shooter, 0.82,
    // 0.67), drivetrain.aimAtTargetCommand(2,0.5)).andThen(new AutoPivotAim(drivetrain, pivot)));

    // robotController.rightBumper().toggleOnTrue(drivetrain.aimAtTargetCommand(2,0.7)
    // .andThen(new AutoPivotAim(drivetrain, pivot))
    // .alongWith(new Shoot(shooter,0.82,0.67)));

    // robotController.rightBumper().toggleOnTrue(new Shoot(shooter, 1, 0.73));
    // TODO: shooter velocity PID control
    // robotController.rightBumper().whileTrue(new ShootSpeaker(shooter,5000,3000));

    // aim at amp and shoot at amp
    // robotController.leftBumper().onTrue(new SequentialCommandGroup(new RotateToAngle(drivetrain,
    // 90, MaxSpeed, 0.3),
    //                                                                 new ParallelCommandGroup(new
    // StrafeToAprilTag(drivetrain, StageAlignment.toleranceDeg)),
    //                                                                                           new
    // SetPivotPos(pivot, PivotConstants.PIVOT_AMP_POS)));

    // run index

    robotController.leftTrigger().whileTrue(new RunIndex(index, INDEX_PCT));
    // robotController
    //     .leftTrigger()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               if (ShooterConstants.IS_AMP) {
    //                 CommandScheduler.getInstance()
    //                     .schedule(
    //                         new Shoot(shooter, index, 0.82, 0.67)
    //                             .andThen(new RunIndex(index, IndexConstants.INDEX_PCT)));

    //               } else {
    //                 CommandScheduler.getInstance().schedule(new Shoot(shooter, index, 0.82,
    // 0.67));
    //               }
    //             }));

    // robotController
    //     .leftTrigger()
    //     .whileFalse(
    //         new ConditionalCommand(
    //             new RunIndex(index, IndexConstants.INDEX_PCT),
    //             Commands.none(),
    //             () -> !ShooterConstants.IS_AMP));

    // run intake
    robotController.rightTrigger().whileTrue(new RunIntake(intake, INTAKE_RUN_PCT). alongWith(new SetPivotPos(pivot, PivotConstants.PIVOT_STOW_POS)));

    // run outtake
    // robotController.povDown().whileTrue(new RunIntake(intake, -INTAKE_RUN_PCT,
    // index::beamBroken));
    robotController.povDown().whileTrue(new RunIndex(index, -0.15). alongWith(new RunIntake(intake, -INTAKE_RUN_PCT)));

    robotController.povUp().whileTrue(new RunIndex(index, IndexConstants.INDEX_PCT));
    // robotController.povLeft().whileTrue(new RunIntake(intake, -INTAKE_RUN_PCT));

    // while the beam break sensor is not broken, run the index
    new Trigger(index::beamBroken)
        .whileFalse(new RunIndex(index, IndexConstants.INDEX_PCT))
        .onTrue(new RunIndex(index, 0));

    // zero the pivot angle at current angle
    // robotController.y().onTrue(new ResetPivotAngle(pivot));

    // Stow Pivot
    robotController.a().onTrue(new SetPivotPos(pivot, PivotConstants.PIVOT_STOW_POS));
    // disables the robot
    // robotController.x().onTrue(Commands.run(() -> CommandScheduler.getInstance().disable()));

    // move shooter up or down
    robotController.povRight().whileTrue(new SetPivotPct(pivot, .5));
    robotController.povLeft().whileTrue(new SetPivotPct(pivot, -.5));

    // Move climb to upright position
    // robotController.povUp().onTrue(new SetPivotPos(pivot, PivotConstants.PIVOT_CLIMB_UP_POS));
    // robotController.povDown().whileTrue(new SetPivotPct(pivot, -0.4));

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

    robotController
        .back()
        .and(robotController.povUp())
        .whileTrue(drivetrain.sysIDQuasistatic(Direction.kForward));
    robotController
        .back()
        .and(robotController.povDown())
        .whileTrue(drivetrain.sysIDQuasistatic(Direction.kReverse));
    robotController
        .start()
        .and(robotController.povUp())
        .whileTrue(drivetrain.sysIDDynamic(Direction.kForward));
    robotController
        .start()
        .and(robotController.povDown())
        .whileTrue(drivetrain.sysIDDynamic(Direction.kReverse));
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
    pipeLineChooser.setDefaultOption("AprilTag 3D", PIPELINE.APRILTAG_3D.value);
    pipeLineChooser.addOption("AprilTag Basic", PIPELINE.APRILTAG_2D.value);
    pipeLineChooser.addOption("Note", PIPELINE.NOTE.value);
    pipeLineChooser.onChange(
        (num) -> {
          drivetrain.getCamera().setPipeline(num);
          System.out.println("Pipeline set to " + num);
        });
    driverTab.add("Pipeline Chooser", pipeLineChooser).withSize(2, 1).withPosition(4, 3);
  }

  // method that configures and initializes everything necessary for auton
  private void setupAutoChooser() {
    NamedCommands.registerCommand(
        "ShootSpeaker", new Shoot(shooter, index, SPK_LEFT_RPM, SPK_RIGHT_RPM));
    NamedCommands.registerCommand(
        "ShootAmp", new Shoot(shooter, index, AMP_LEFT_SPEED, AMP_RIGHT_SPEED));
    // NamedCommands.registerCommand("RunIntake", new RunIntake(intake, INTAKE_RUN_PCT));
    NamedCommands.registerCommand("RunIntake", Commands.print("RunIntake"));
    NamedCommands.registerCommand(
        "AimAtTarget", new AimAtTarget(drivetrain, StageAlignment.toleranceDeg));
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
