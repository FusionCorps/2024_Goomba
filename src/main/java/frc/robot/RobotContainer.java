// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.IntakeConstants.INTAKE_RUN_PCT;
import static frc.robot.Constants.LimelightConstants.BLUE_SPK_TAG_ID;
import static frc.robot.Constants.LimelightConstants.RED_SPK_TAG_ID;
import static frc.robot.Constants.PivotConstants.PIVOT_CLIMB_DOWN_POS;
import static frc.robot.Constants.PivotConstants.PIVOT_STOW_POS;
import static frc.robot.Constants.PivotConstants.PIVOT_SUB_POS;
import static frc.robot.Constants.PivotConstants.PIVOT_TRAP_POS;
import static frc.robot.Constants.ShooterConstants.IS_AMP;
import static frc.robot.Constants.ShooterConstants.SHOOTER_OUTTAKE_RPM;
import static frc.robot.Constants.ShooterConstants.SPK_LEFT_RPM;
import static frc.robot.Constants.ShooterConstants.SPK_RIGHT_RPM;
import static frc.robot.Constants.allianceColor;
import static frc.robot.Constants.driverTab;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants.PIPELINE;
import frc.robot.Constants.StageAlignment;
import frc.robot.Constants.TransferHookConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.TransferHooks.HoldHooks;
import frc.robot.commands.TransferHooks.SetHooksPos;
import frc.robot.commands.index.IndexDummy;
import frc.robot.commands.index.Outtake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.pivot.AutoPivotAim;
import frc.robot.commands.pivot.HoldPivotAngle;
import frc.robot.commands.pivot.SetAngleAmp;
import frc.robot.commands.pivot.SetAngleShuttle;
import frc.robot.commands.pivot.SetPivotPct;
import frc.robot.commands.pivot.SetPivotPos;
import frc.robot.commands.pivot.TrapPivot;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootAuto;
import frc.robot.commands.shooter.StopRevShooter;
import frc.robot.commands.swerve.manual.RunSwerveFC;
import frc.robot.commands.swerve.vision.AimAtTarget;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TransferHooks;

public class RobotContainer {
  public static CommandXboxController robotController = new CommandXboxController(0);
  Drivetrain drivetrain = Constants.DrivetrainConstants.DriveTrain;

  Intake intake = new Intake();
  Shooter shooter = new Shooter();
  Index index = new Index();
  TransferHooks transferHooks = new TransferHooks();
  Pivot pivot = new Pivot();

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private SendableChooser<Integer> pipeLineChooser = new SendableChooser<>();

  /**
   * Configures the bindings for the robot's subsystems and commands.
   *
   * <ul>
   *   <li>Left/Right sticks - field-centric swerve drive
   *   <li>B: reset gyro
   *   <li>RB: aim drivetrain + pivot at speaker
   *   <li>RT: intake note + stow pivot
   *   <li>LT: rev shooter on hold + shoot note on release
   *   <li>Y - manually stop revving shooter
   *   <li>A: stow pivot
   *   <li>POV up/down - manually move pivot
   *   <li>LB: aim pivot at amp
   *   <li>X: aim pivot for shuttling
   *   <li>POV left - outtake thru intake
   *   <li>POV right - outtake thru shooter
   *   <li>Start - Aims pivot for climbing
   *   <li>Back - Runs climb routine
   * </ul>
   */
  private void configureBindings() {
    // Run field-centric swerve drive
    drivetrain.setDefaultCommand(new RunSwerveFC(drivetrain));

    // transferHooks.setDefaultCommand(new HoldHooks(transferHooks));

    // Reset odometry to current position, and zero gyro yaw
    robotController
        .b()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  drivetrain.seedFieldRelative();
                //   drivetrain.resetGyro();
                }));

    // Aims in-place at april tag
    robotController
        .rightBumper()
        .toggleOnTrue(
            new AimAtTarget(drivetrain, StageAlignment.toleranceDeg, () -> !index.beamBroken())
                .alongWith(new AutoPivotAim(pivot, drivetrain.getCamera(), index, PIVOT_STOW_POS)));

    // robotController.rightBumper().onTrue(new SetPivotPos(pivot,
    // 36.943115234375));

    // Run intake mechanism
    robotController
        .rightTrigger()
        .whileTrue(new RevShooter(shooter, 0, 0).alongWith(new IntakeNote(intake, index, pivot)));

    // Shoots note at speaker
    robotController
        .leftTrigger()
        .onTrue(new RevShooter(shooter, SPK_LEFT_RPM, SPK_RIGHT_RPM))
        .onFalse(new IndexDummy(index).withTimeout(2).andThen(new StopRevShooter(shooter)));

    // robotController.leftTrigger().whileTrue(new Trap(index));

    // Manually stop revving the shooter
    robotController.y().onTrue(new StopRevShooter(shooter));

    // Set to Subwoofer Shot
    robotController.a().onTrue(new SetPivotPos(pivot, PIVOT_SUB_POS));

    // Move pivot up/down
    robotController.povUp().whileTrue(new SetPivotPct(pivot, index, -.4));
    robotController.povUp().onFalse(new HoldPivotAngle(pivot));

    robotController.povDown().whileTrue(new SetPivotPct(pivot, index, .35));
    robotController.povDown().onFalse(new HoldPivotAngle(pivot));

    // Sets pivot to angle for Amp scoring
    robotController.leftBumper().onTrue(new SetAngleAmp(pivot));

    // Sets Pivot to angle for Shuttling
    robotController.x().onTrue(new SetAngleShuttle(pivot));

    // Outtake thru intake
    robotController
        .povLeft()
        .whileTrue(
            new SetPivotPos(pivot, PIVOT_STOW_POS)
                .alongWith(new Outtake(index))
                .alongWith(new RunIntake(intake, -INTAKE_RUN_PCT)));

    // Outtake thru shooter
    robotController
        .povRight()
        .whileTrue(
            new RevShooter(shooter, SHOOTER_OUTTAKE_RPM, SHOOTER_OUTTAKE_RPM)
                .alongWith(new IndexDummy(index)))
        .onFalse(new StopRevShooter(shooter));

    // robotController.start().whileTrue(new SetHooksPct(transferHooks, 0.3));

    // robotController.start().onTrue(new Climb(pivot, transferHooks, index));

    robotController
        .start()
        .onTrue(
            new TrapPivot(pivot, PIVOT_CLIMB_DOWN_POS)
                .andThen(
                    new HoldPivotAngle(pivot)
                        .alongWith(
                            new SetHooksPos(
                                transferHooks, TransferHookConstants.TRANSFER_HOOK_POS_CLIMB)))
                .andThen(
                    new HoldHooks(transferHooks).alongWith(new TrapPivot(pivot, PIVOT_TRAP_POS))));

    // xx().whileTrue(new SetHooksPct(transferHooks, -0.3));

    robotController.back().onTrue(new TrapPivot(pivot, PIVOT_TRAP_POS));
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
        "RevShooterLoad", new RevShooter(shooter, SPK_LEFT_RPM, SPK_RIGHT_RPM));
    NamedCommands.registerCommand(
        "RevShooterAmp", new RevShooter(shooter, SPK_RIGHT_RPM, SPK_LEFT_RPM));

    // hardcoded manual aiming angles
    NamedCommands.registerCommand("PivotAimLoadStart", new SetPivotPos(pivot, 33.84));
    NamedCommands.registerCommand("PivotAimAmpStart", new SetPivotPos(pivot, 36.943115234375));
    NamedCommands.registerCommand("PivotAimLoadStage", new SetPivotPos(pivot, 17.75));
    NamedCommands.registerCommand("PivotAimP1456/P1564-Far", new SetPivotPos(pivot, 12.763));
    NamedCommands.registerCommand("PivotAimSecondFar", new SetPivotPos(pivot, 9.8));

    NamedCommands.registerCommand(
        "AimPivot", new AutoPivotAim(pivot, drivetrain.getCamera(), index, PIVOT_STOW_POS));

    NamedCommands.registerCommand(
        "AimSwerveAndPivotFirst",
        new ParallelDeadlineGroup(
            new AimAtTarget(drivetrain, StageAlignment.toleranceDeg, () -> !index.beamBroken())
                .withTimeout(0.28),
            new AutoPivotAim(pivot, drivetrain.getCamera(), index, PIVOT_STOW_POS)));
    NamedCommands.registerCommand(
        "AimSwerveAndPivot",
        new ParallelDeadlineGroup(
            new AimAtTarget(drivetrain, StageAlignment.toleranceDeg, () -> !index.beamBroken())
                .withTimeout(0.2),
            new AutoPivotAim(pivot, drivetrain.getCamera(), index, PIVOT_STOW_POS)));

    NamedCommands.registerCommand(
        "AimSwerveAndPivotLong",
        new ParallelDeadlineGroup(
            new AimAtTarget(drivetrain, StageAlignment.toleranceDeg, () -> !index.beamBroken())
                .withTimeout(0.5),
            new AutoPivotAim(pivot, drivetrain.getCamera(), index, PIVOT_STOW_POS)));

    NamedCommands.registerCommand(
        "AimSwerveAndPivotLongLast",
        new ParallelDeadlineGroup(
            new AimAtTarget(drivetrain, StageAlignment.toleranceDeg, () -> !index.beamBroken())
                .withTimeout(0.8),
            new AutoPivotAim(pivot, drivetrain.getCamera(), index, PIVOT_STOW_POS)));

    NamedCommands.registerCommand(
        "AimPivotAndShoot",
        new AutoPivotAim(pivot, drivetrain.getCamera(), index, PIVOT_STOW_POS)
            .andThen(new ShootAuto(index)));
    NamedCommands.registerCommand(
        "AimAtTarget",
        new AimAtTarget(drivetrain, StageAlignment.toleranceDeg, () -> !index.beamBroken())
            .withTimeout(.2));

    NamedCommands.registerCommand("ShootSpeaker", new Shoot(index, shooter));
    NamedCommands.registerCommand("ShootAuto", new ShootAuto(index));
    NamedCommands.registerCommand(
        "ShootAmp", Commands.runOnce(() -> IS_AMP = true).andThen(new Shoot(index, shooter)));

    NamedCommands.registerCommand("IntakeNote", new IntakeNote(intake, index, pivot));
    NamedCommands.registerCommand(
        "StopIntake",
        new InstantCommand(
            () -> {
              IntakeConstants.IS_INTAKING = false;
            }));

    System.out.println(AutoBuilder.getAllAutoNames());
    // if this throws an error, make sure all autos are complete
    // can verify what paths/autos are on rio: ftp://roboRIO-6672-frc.local
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    autoChooser.addOption("3 Piece Load Side Blue", AutoBuilder.buildAuto("Auto3-P87Blue"));
    // autoChooser.addOption("4 Piece Load Side Far Blue",
    // AutoBuilder.buildAuto("Auto4-P873Blue"));
    // autoChooser.addOption("4 Piece Amp Side Far Blue",
    // AutoBuilder.buildAuto("Auto4-P146Blue"));
    // autoChooser.addOption("4 Piece Load Side Close Blue",
    // AutoBuilder.buildAuto("Auto4-P321Blue"));
    autoChooser.addOption("4 Piece Load Side Far Blue", AutoBuilder.buildAuto("Auto4-P876Blue"));
    autoChooser.addOption("5 Piece Amp Side Far Blue", AutoBuilder.buildAuto("Auto5-P1456Blue"));
    autoChooser.addOption("6 Piece Amp Side Far Blue", AutoBuilder.buildAuto("Auto6-P125643Blue"));

    autoChooser.addOption("3 Piece Load Side Red", AutoBuilder.buildAuto("Auto3-P87Red"));
    // autoChooser.addOption("4 Piece Load Side Far Red",
    // AutoBuilder.buildAuto("Auto4-P873Red"));
    // autoChooser.addOption("4 Piece Amp Side Far Red",
    // AutoBuilder.buildAuto("Auto4-P146Red"));
    // autoChooser.addOption("4 Piece Load Side Close Red",
    // AutoBuilder.buildAuto("Auto4-P321Red"));
    autoChooser.addOption("4 Piece Load Side Far Red", AutoBuilder.buildAuto("Auto4-P876Red"));
    autoChooser.addOption("5 Piece Amp Side Far Red", AutoBuilder.buildAuto("Auto5-P1456Red"));
    autoChooser.addOption("6 Piece Amp Side Far Red", AutoBuilder.buildAuto("Auto6-P125643Red"));

    // TODO: test using conditional commands
    // autoChooser.addOption("3 Piece Load Side", new ConditionalCommand(
    // AutoBuilder.buildAuto("Auto3-P87Red"),
    // AutoBuilder.buildAuto("Auto3-P87Blue"),
    // drivetrain::isAllianceRed));

    // autoChooser.addOption("4 Piece Load Side", new ConditionalCommand(
    // AutoBui~lder.buildAuto("Auto4-P873Red"),
    // AutoBuilder.buildAuto("Auto4-P873Blue"),
    // drivetrain::isAllianceRed));

    // autoChooser.addOption("4 Piece Amp Side Mid", new ConditionalCommand(
    // AutoBuilder.buildAuto("Auto4-P146Red"),
    // AutoBuilder.buildAuto("Auto4-P146Red"),
    // drivetrain::isAllianceRed));

    // autoChooser.addOption("Testing-ForwardAuto",
    // AutoBuilder.buildAuto("ForwardAuto"));

    driverTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(4, 2);
  }

  public RobotContainer() {
    setupAutoChooser();
    setupPipelineChooser();
    setupAllianceColorChooser();
    configureBindings();
  }

  private void setupAllianceColorChooser() {
    SendableChooser<DriverStation.Alliance> colorChooser = new SendableChooser<>();

    colorChooser.addOption("Red", Alliance.Red);
    colorChooser.addOption("Blue", Alliance.Blue);
    colorChooser.setDefaultOption(allianceColor.toString(), allianceColor);
    colorChooser.onChange(
        color -> {
          allianceColor = color;
          drivetrain
              .getCamera()
              .setPriorityID(color == Alliance.Blue ? BLUE_SPK_TAG_ID : RED_SPK_TAG_ID);
        });
    driverTab.add("Alliance Color Chooser", colorChooser).withSize(2, 1).withPosition(4, 4);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
