// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.IntakeConstants.INTAKE_RUN_PCT;
import static frc.robot.Constants.PivotConstants.PIVOT_CLIMB_DOWN_POS;
import static frc.robot.Constants.PivotConstants.PIVOT_STOW_POS;
import static frc.robot.Constants.PivotConstants.PIVOT_SUB_POS;
import static frc.robot.Constants.PivotConstants.PIVOT_TRAP_POS;
import static frc.robot.Constants.ShooterConstants.IS_AMP;
import static frc.robot.Constants.ShooterConstants.SHOOTER_OUTTAKE_RPM;
import static frc.robot.Constants.ShooterConstants.SPK_LEFT_RPM;
import static frc.robot.Constants.ShooterConstants.SPK_RIGHT_RPM;
import static frc.robot.Constants.TransferHookConstants.TRANSFER_HOOK_POS_CLIMB;
import static frc.robot.Constants.allianceColor;
import static frc.robot.Constants.driverTab;
import static frc.robot.Constants.IndexConstants.INDEX_RUN_PCT;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LimelightConstants.PIPELINE;
import frc.robot.Constants.StageAlignment;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.TransferHooks.HoldHooks;
import frc.robot.commands.TransferHooks.SetHooksPct;
import frc.robot.commands.TransferHooks.SetHooksPos;
import frc.robot.commands.index.IndexDummy;
import frc.robot.commands.index.Outtake;
import frc.robot.commands.index.RunIndex;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.pivot.AutoPivotAim;
import frc.robot.commands.pivot.HoldPivotAngle;
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
   * Configures the bindings for the robot's subsystems and commands.
   *
   * <ul>
   * <li>Left/Right sticks - field-centric swerve drive
   * <li>B: reset gyro
   * <li>RB: aim drivetrain + pivot at speaker
   * <li>RT: intake note + stow pivot
   * <li>LT: rev shooter on hold + shoot note on release
   * <li>Y - manually stop revving shooter
   * <li>A: stow pivot
   * <li>POV up/down - manually move pivot
   * <li>LB: aim pivot at amp
   * <li>X: aim pivot for shuttling
   * <li>POV left - outtake thru intake
   * <li>POV right - outtake thru shooter
   * <li>Start - Aims pivot for climbing
   * <li>Back - Runs climb routine
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
                  drivetrain.resetGyro();
                }));

    // Aims in-place at april tag
    robotController
        .rightBumper()
        .toggleOnTrue(
            new AimAtTarget(drivetrain, StageAlignment.toleranceDeg, () -> !index.beamBroken())
                .alongWith(new AutoPivotAim(pivot, drivetrain.getCamera(), index)));

    // Run intake mechanism
    robotController
        .rightTrigger()
        .whileTrue( new RevShooter(shooter, 0, 0). alongWith(
            new IntakeNote(intake, index, pivot)));

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
    robotController.povDown().whileTrue(new SetPivotPct(pivot, index, .35));

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
                .alongWith(new RunIndex(index, -INDEX_RUN_PCT)))
        .onFalse(new StopRevShooter(shooter));

    // robotController.start().whileTrue(new SetHooksPct(transferHooks, 0.5));
    // robotController.back().whileTrue(new SetHooksPct(transferHooks, -0.5));

    // Ready climb
    // robotController.start().onTrue(
    //     new SetPivotPos(pivot, PIVOT_CLIMB_DOWN_POS)
    //         .andThen(new HoldPivotAngle(pivot).alongWith(new SetHooksPos(transferHooks, TRANSFER_HOOK_POS_CLIMB)))
    //         .andThen(new HoldHooks(transferHooks)));
    // Auto Climb
    // robotController.back().onTrue(new SetPivotPos(pivot, PIVOT_TRAP_POS));

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
    // .rightTrigger()
    // .whileTrue(
    // new SetPivotPos(pivot, PivotConstants.PIVOT_STOW_POS)
    // .alongWith(new Shoot(index))
    // .alongWith(new RunIntake(intake, INTAKE_RUN_PCT))
    // .alongWith(new RevShooter(shooter, SPK_LEFT_RPM, SPK_RIGHT_RPM))).onFalse(new
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
    NamedCommands.registerCommand("PivotAimP1456/P1564-1", new SetPivotPos(pivot, 17.75));
    NamedCommands.registerCommand("PivotAimP1456/P1564-Far", new SetPivotPos(pivot, 10.7));
    NamedCommands.registerCommand("PivotAimSecondFar", new SetPivotPos(pivot, 9.8));

    NamedCommands.registerCommand(
        "AimAndShoot",
        new AutoPivotAim(pivot, drivetrain.getCamera(), index).andThen(new Shoot(index, shooter)));

    NamedCommands.registerCommand("ShootSpeaker", new Shoot(index, shooter));
    NamedCommands.registerCommand(
        "ShootAmp", Commands.runOnce(() -> IS_AMP = true).andThen(new Shoot(index, shooter)));

    NamedCommands.registerCommand("IntakeNote", new IntakeNote(intake, index, pivot));

    System.out.println(AutoBuilder.getAllAutoNames());
    // if this throws an error, make sure all autos are complete
    // can verify what paths/autos are on rio: ftp://roboRIO-6672-frc.local
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    
      // autoChooser.addOption("Comp-4 Piece load side", AutoBuilder.buildAuto("Auto4-P873Red"));
      // autoChooser.addOption("Comp-4 Piece center", AutoBuilder.buildAuto("Auto4-P321Red"));
      // autoChooser.addOption(
      // "Comp-5 Piece Top First Mid", Commands.print("Auto5-P1456Red")); // TODO: add
      // autoChooser.addOption(
      // "Comp-5 Piece Top Last Mid", Commands.print("Auto5-P1564Red")); // TODO: add
      // autoChooser.addOption(
          // "Comp 4 Piece Amp Side Mid", Commands.print("Auto4-P146Red")); // TODO: add
    
      autoChooser.addOption("Comp-4 Piece load side", AutoBuilder.buildAuto("Auto4-P873Blue"));
      autoChooser.addOption("Comp-4 Piece center", AutoBuilder.buildAuto("Auto4-P321Blue"));
      // autoChooser.addOption("Comp-5 Piece Top First Mid",
      // AutoBuilder.buildAuto("Auto5-P1456Blue"));
      // autoChooser.addOption("Comp-5 Piece Top Last Mid",
      // AutoBuilder.buildAuto("Auto5-P1564Blue"));
      autoChooser.addOption("Comp 4 Piece Amp Side Mid", AutoBuilder.buildAuto("Auto4-P146Blue"));
    

    autoChooser.addOption("Testing-ForwardAuto",
    AutoBuilder.buildAuto("ForwardAuto"));
    // autoChooser.addOption("Testing-StrafeAuto",
    // AutoBuilder.buildAuto("StrafeAuto"));
    // autoChooser.addOption("Testing-RotationAuto",
    // AutoBuilder.buildAuto("RotationAuto"));

    driverTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(4, 2);
  }

  public RobotContainer() {
    setupAutoChooser();
    setupPipelineChooser();
    SendableChooser<DriverStation.Alliance> colorChooser = new SendableChooser<>();

    colorChooser.addOption("Red", Alliance.Red);
    colorChooser.addOption("Blue", Alliance.Blue);
    colorChooser.setDefaultOption(allianceColor.toString(), allianceColor);
    colorChooser.onChange(color -> {
      allianceColor = color;
      drivetrain.getCamera().setPriorityID(color == Alliance.Blue ? 7 : 4);
    });
    driverTab.add("Alliance Color Chooser", colorChooser).withSize(2, 1).withPosition(4, 4);
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
