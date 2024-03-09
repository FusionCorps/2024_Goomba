// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ShooterConstants.SPK_LEFT_RPM;
import static frc.robot.Constants.ShooterConstants.SPK_RIGHT_RPM;
import static frc.robot.Constants.driverTab;
import static frc.robot.Constants.IndexConstants.INDEX_PCT;
import static frc.robot.Constants.IntakeConstants.INTAKE_RUN_PCT;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LimelightConstants.PIPELINE;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StageAlignment;
import frc.robot.commands.AimAtPoint;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.TransferHooks.SetHooksPct;
import frc.robot.commands.index.RunIndex;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.pivot.AutoPivotAim;
import frc.robot.commands.pivot.DownClimbPos;
import frc.robot.commands.pivot.SetPivotPct;
import frc.robot.commands.pivot.SetPivotPos;
import frc.robot.commands.pivot.UpClimbPos;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.StopRevShooter;
import frc.robot.commands.swerve.manual.RunSwerveFC;
import frc.robot.commands.swerve.vision.AimAtTarget;
import frc.robot.commands.swerve.vision.RotateToAngle;
import frc.robot.subsystems.*;

public class RobotContainer {
  public static CommandXboxController robotController = new CommandXboxController(0);
  public Drivetrain drivetrain = Constants.DrivetrainConstants.DriveTrain;

  public Intake intake = new Intake();
  public Shooter shooter = new Shooter();
  public Index index = new Index();
  public Pivot pivot = new Pivot();
  public TransferHooks hooks = new TransferHooks();

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private SendableChooser<Integer> pipeLineChooser = new SendableChooser<>();

  /**
   * Configures the bindings for the robot's subsystems and commands. LT: rev up shooter, releasing
   * shooter RT: intake + stow RB: aim w speaker LB: aim with amp/trap A: stow pivot B: reset gyro
   * POV up/down - move pivot POV right - outtake thru intake POV left - outtake thru shooter Start
   * - Up climb pos Back - Down climb pos
   */
  private void configureBindings() {

    // run field centric swerve drive
    drivetrain.setDefaultCommand(new RunSwerveFC(drivetrain));

    // index.setDefaultCommand(new RunIndex(index, IndexConstants.INDEX_PCT));

    // reset odometry to current position, and zero gyro yaw
    robotController
        .b()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  drivetrain.seedFieldRelative();
                  drivetrain.resetGyro();
                }));

    // // outake through intake
    robotController.povLeft().whileTrue(new SetPivotPos(pivot,
    PivotConstants.PIVOT_STOW_POS)
    .alongWith(new RunIndex(index, -INDEX_PCT)).alongWith(new RunIntake(intake,
    -INTAKE_RUN_PCT)));

    // // outake through shooter
    robotController.povRight().whileTrue(
    new RevShooter(shooter, ShooterConstants.SHOOTER_OUTTAKE_RPM,
    ShooterConstants.SHOOTER_OUTTAKE_RPM).alongWith(new
    Shoot(shooter, index)));

    // robotController.povRight().whileTrue(new SetHooksPct(hooks, 0.5));
    // robotController.povLeft().whileTrue(new SetHooksPct(hooks, -0.5));

    robotController
        .rightBumper()
        .toggleOnTrue(
            new AimAtTarget(drivetrain, StageAlignment.toleranceDeg)
                .alongWith(new AutoPivotAim(pivot, drivetrain.getCamera())));

    // aim at amp and shoot at amp
    // robotController.leftBumper().onTrue(new SequentialCommandGroup(new
    // RotateToAngle(drivetrain,
    // 90, MaxSpeed, 0.3),
    // new ParallelCommandGroup(new
    // StrafeToAprilTag(drivetrain, StageAlignment.toleranceDeg)),
    // new
    // SetPivotPos(pivot, PivotConstants.PIVOT_AMP_POS)));

    // TODO: test aim with localization
    robotController.leftBumper().toggleOnTrue(new AimAtPoint(drivetrain, Constants.redSpeakerPos));

    robotController
        .leftTrigger()
        .onTrue(new RevShooter(shooter, SPK_LEFT_RPM, SPK_RIGHT_RPM))
        .onFalse(new Shoot(shooter, index).withTimeout(0.2).andThen(new StopRevShooter(shooter)));
    // .onFalse(
    // new RunIndex(index, IndexConstants.INDEX_PCT).withTimeout(0.2).andThen(new
    // RevShooter(shooter, 0, 0)));

    // robotController.leftTrigger()
    // .onFalse(new Shoot(shooter, index).withTimeout(0.03).andThen(new
    // RevShooter(shooter, 0, 0)));

    // robotController.leftTrigger().whileTrue(new RunIndex(index, INDEX_PCT));

    // run complete intake mechanism
    robotController.rightTrigger().whileTrue(new IntakeNote(intake, index, pivot));

    // robotController.rightTrigger().whileTrue(new RunIntake(intake, INTAKE_RUN_PCT));

    // cancel reving the shooter
    robotController.y().onTrue(new StopRevShooter(shooter));

    // robotController.y().onTrue(new RevShooter(shooter, 3000, 2000));
    // robotController.x().onTrue(new RevShooter(shooter, 0, 0));

    // Stow Pivot
    robotController.a().onTrue(new SetPivotPos(pivot, PivotConstants.PIVOT_STOW_POS));

    // puts robot in shuttling mode
    // robotController.x().onTrue(new Shuttling(pivot));

    // move shooter up or down manually
    robotController.povUp().whileTrue(new SetPivotPct(pivot, -.1));
    robotController.povDown().whileTrue(new SetPivotPct(pivot, .1));

    // Move climb to upright position
    robotController.start().onTrue(new UpClimbPos(pivot));
    // Move climb to down position
    robotController.back().onTrue(new DownClimbPos(pivot));

    // // rotate in
    // place to aim at target

    // rotate to perpendicular with wall, strafe to center on april tag, and rumble
    // robotController

    // .x()
    // .whileTrue(
    // drivetrain
    // .rotateToAngleCommand(180, 2.0, 0.75)
    // // new RotateToAngle(drivetrain, 180, 2.0, 0.75)
    // .andThen(Commands.print("rotation finished"))
    // .andThen(drivetrain.strafeToAprilTagCommand(2.0))
    // // .andThen(new StrafeToAprilTag(drivetrain, 2.0))
    // .andThen(Commands.print("strafe finished"))
    // .andThen(
    // Commands.runOnce(
    // () -> robotController.getHID().setRumble(RumbleType.kBothRumble, 0.2)))
    // .andThen(new WaitCommand(0.1))
    // .andThen(
    // Commands.runOnce(
    // () -> robotController.getHID().setRumble(RumbleType.kBothRumble, 0.0))));

  }

  public RobotContainer() {
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
        "RevShooter", new RevShooter(shooter, SPK_LEFT_RPM, SPK_RIGHT_RPM));
    NamedCommands.registerCommand("ShootSpeaker", new Shoot(shooter, index));
    NamedCommands.registerCommand("ShootAmp", new Shoot(shooter, index)); // TODO: fix
    NamedCommands.registerCommand("IntakeNote", new IntakeNote(intake, index, pivot));
    NamedCommands.registerCommand(
        "Rotate180",
        new RotateToAngle(drivetrain, 180.0, StageAlignment.toleranceDeg).withTimeout(1.0));
    NamedCommands.registerCommand(
        "AimAndShoot",
        (new AimAtTarget(drivetrain, StageAlignment.toleranceDeg)
                .alongWith(new AutoPivotAim(pivot, drivetrain.getCamera())))
            .withTimeout(1.0)
            .andThen(new Shoot(shooter, index)));

    System.out.println(AutoBuilder.getAllAutoNames());
    // if this throws an error, make sure all autos are complete
    // can verify what paths/autos are on rio: ftp://roboRIO-6672-frc.local
    autoChooser = AutoBuilder.buildAutoChooser();
    // autoChooser = new SendableChooser<>();
    // autoChooser.setDefaultOption("Do Nothing", Commands.none());
    // autoChooser.addOption("ForwardAuto", AutoBuilder.buildAuto("ForwardAuto"));
    // autoChooser.addOption("StrafeAuto", AutoBuilder.buildAuto("StrafeAuto"));
    // autoChooser.addOption("RotationAuto", AutoBuilder.buildAuto("RotationAuto"));

    // autoChooser.addOption("Auto1STopF", AutoBuilder.buildAuto("Auto1STopF"));
    // autoChooser.addOption("Auto3STopF", AutoBuilder.buildAuto("Auto3STopF"));
    // autoChooser.addOption("Auto4-P321", AutoBuilder.buildAuto("Auto4-P321"));
    // autoChooser.addOption("Auto4-P873", AutoBuilder.buildAuto("Auto4-P873"));
    // autoChooser.addOption("Auto5-P1456", AutoBuilder.buildAuto("Auto5-P1456"));
    // autoChooser.addOption("Auto5-P1564", AutoBuilder.buildAuto("Auto5-P1564"));

    driverTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(4, 2);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
