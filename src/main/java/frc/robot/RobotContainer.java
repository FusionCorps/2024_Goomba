// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.IndexConstants.INDEX_RUN_PCT;
import static frc.robot.Constants.IntakeConstants.INTAKE_RUN_PCT;
import static frc.robot.Constants.PivotConstants.PIVOT_READY_CLIMB_POS;
import static frc.robot.Constants.PivotConstants.PIVOT_STOW_POS;
import static frc.robot.Constants.PivotConstants.PIVOT_SUB_POS;
import static frc.robot.Constants.ShooterConstants.IS_AMP;
import static frc.robot.Constants.ShooterConstants.SHOOTER_OUTTAKE_RPM;
import static frc.robot.Constants.ShooterConstants.SPK_LEFT_RPM;
import static frc.robot.Constants.ShooterConstants.SPK_RIGHT_RPM;
import static frc.robot.Constants.driverTab;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LimelightConstants.PIPELINE;
import frc.robot.Constants.StageAlignment;
import frc.robot.commands.Climb;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.index.RunIndex;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.pivot.AutoPivotAim;
import frc.robot.commands.pivot.SetAngleAmp;
import frc.robot.commands.pivot.SetAngleShuttle;
import frc.robot.commands.pivot.SetPivotPct;
import frc.robot.commands.pivot.SetPivotPos;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.commands.shooter.Shoot;
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
   * Configures the bindings for the robot's subsystems and commands. LT: rev up shooter, releasing
   * shooter RT: intake + stow RB: aim w speaker LB: aim with amp/trap A: stow pivot B: reset gyro
   * POV up/down - move pivot POV right - outtake thru intake POV left - outtake thru shooter Start
   * - Up climb pos Back - Down climb pos
   */
  private void configureBindings() {
    // Run field-centric swerve drive
    drivetrain.setDefaultCommand(new RunSwerveFC(drivetrain));

    // Reset odometry to current position, and zero gyro yaw
    robotController
        .b()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  drivetrain.seedFieldRelative();
                  drivetrain.resetGyro();
                }));

    // Aims in-place at april tag
    robotController
        .rightBumper()
        .toggleOnTrue(
            new AimAtTarget(drivetrain, StageAlignment.toleranceDeg, () -> !index.beamBroken())
                .alongWith(new AutoPivotAim(pivot, drivetrain.getCamera(), index)));

    // Run intake mechanism
    robotController.rightTrigger().whileTrue(new IntakeNote(intake, index, pivot));

    // Shoots note at speaker
    robotController
        .leftTrigger()
        .onTrue(new RevShooter(shooter, SPK_LEFT_RPM, SPK_RIGHT_RPM))
        .onFalse(new Shoot(index).withTimeout(0.8).andThen(new StopRevShooter(shooter)));

    // Manually stop revving the shooter
    robotController.povRight().onFalse(new StopRevShooter(shooter));

    // Set to Subwoofer Shot
    robotController.a().onTrue(new SetPivotPos(pivot, PIVOT_SUB_POS));

    // Move pivot up/down
    robotController.povUp().whileTrue(new SetPivotPct(pivot, -.4));
    robotController.povDown().whileTrue(new SetPivotPct(pivot, .15));

    // Sets pivot to angle for Amp scoring
    robotController.leftBumper().onTrue(new SetAngleAmp(pivot));

    // Sets Pivot to angle for Shuttling
    robotController.x().onTrue(new SetAngleShuttle(pivot));

    // Outtake thru intake
    robotController
        .povLeft()
        .whileTrue(
            new SetPivotPos(pivot, PIVOT_STOW_POS)
                .alongWith(new RunIndex(index, -INDEX_RUN_PCT))
                .alongWith(new RunIntake(intake, -INTAKE_RUN_PCT)));

    // Outtake thru shooter
    robotController
        .povRight()
        .whileTrue(
            new RevShooter(shooter, SHOOTER_OUTTAKE_RPM, SHOOTER_OUTTAKE_RPM)
                .alongWith(new Shoot(index)));

    // robotController.start().whileTrue(new SetHooksPct(transferHooks, 0.5));
    // robotController.back().whileTrue(new SetHooksPct(transferHooks, -0.5));

    // Ready climb
    robotController.start().onTrue(new SetPivotPos(pivot, PIVOT_READY_CLIMB_POS));
    // Auto Climb
    robotController.back().onTrue(new Climb(pivot, drivetrain, transferHooks, index));

    // aim at amp and shoot at amp
    // robotController.leftBumper().onTrue(new SequentialCommandGroup(new
    // RotateToAngle(drivetrain,
    // 90, MaxSpeed, 0.3),
    // new ParallelCommandGroup(new
    // StrafeToAprilTag(drivetrain, StageAlignment.toleranceDeg)),
    // new
    // SetPivotPos(pivot, PivotConstants.PIVOT_AMP_POS)));

    // robotController.leftBumper().whileTrue(new RevShooter(shooter, -1200,
    // -1200));
    // robotController.leftBumper()
    // .onFalse(new RunIndex(index, 0.23).withTimeout(1).andThen(new
    // RevShooter(shooter, 0, 0)));

    // Continuous intake+shooting motion
    // robotController
    //     .rightTrigger()
    //     .whileTrue(
    //         new SetPivotPos(pivot, PivotConstants.PIVOT_STOW_POS)
    //             .alongWith(new Shoot(index))
    //             .alongWith(new RunIntake(intake, INTAKE_RUN_PCT))
    //             .alongWith(new RevShooter(shooter, SPK_LEFT_RPM, SPK_RIGHT_RPM))).onFalse(new
    // StopRevShooter(shooter));

    // Move climb to upright position
    // robotController.start().onTrue(new UpClimbPos(pivot));
    // // Move climb to down position
    // robotController.back().onTrue(new DownClimbPos(pivot));

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
        "RevShooterLoadSide", new RevShooter(shooter, SPK_LEFT_RPM, SPK_RIGHT_RPM));
    NamedCommands.registerCommand(
        "RevShooterAmpSide", new RevShooter(shooter, SPK_RIGHT_RPM, SPK_LEFT_RPM));

    NamedCommands.registerCommand("StartPosLoad", new SetPivotPos(pivot, 33.84));
    NamedCommands.registerCommand("StartPosAmp", new SetPivotPos(pivot, 36.943115234375));
    NamedCommands.registerCommand("PivotAimP1456/P1564-1", new SetPivotPos(pivot, 19.55));
    NamedCommands.registerCommand("PivotAimP1456/P1564-Far", new SetPivotPos(pivot, 13));

    NamedCommands.registerCommand(
        "AimAndShoot",
        new AutoPivotAim(pivot, drivetrain.getCamera(), index).andThen(new Shoot(index)));

    NamedCommands.registerCommand("ShootSpeaker", new Shoot(index));
    NamedCommands.registerCommand(
        "ShootAmp", Commands.runOnce(() -> IS_AMP = true).andThen(new Shoot(index)));

    NamedCommands.registerCommand("IntakeNote", new IntakeNote(intake, index, pivot));

    System.out.println(AutoBuilder.getAllAutoNames());
    // if this throws an error, make sure all autos are complete
    // can verify what paths/autos are on rio: ftp://roboRIO-6672-frc.local
    // autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("ForwardAuto", AutoBuilder.buildAuto("ForwardAuto"));
    autoChooser.addOption("StrafeAuto", AutoBuilder.buildAuto("StrafeAuto"));
    autoChooser.addOption("RotationAuto", AutoBuilder.buildAuto("RotationAuto"));
    autoChooser.addOption("4 Piece far side", AutoBuilder.buildAuto("Auto4-P873"));
    autoChooser.addOption("Close 4 Piece", AutoBuilder.buildAuto("Auto4-P321"));
    autoChooser.addOption("1456", AutoBuilder.buildAuto("Auto5-P1456"));
    autoChooser.addOption("1564", AutoBuilder.buildAuto("Auto5-P1564"));

    autoChooser.addOption("Auto1STopF", AutoBuilder.buildAuto("Auto1STopF"));
    autoChooser.addOption("Auto3STopF", AutoBuilder.buildAuto("Auto3STopF"));

    driverTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(4, 2);
  }

  public RobotContainer() {
    setupAutoChooser();
    setupPipelineChooser();
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
