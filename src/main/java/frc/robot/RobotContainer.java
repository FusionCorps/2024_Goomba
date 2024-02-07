// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.shooter.AimShooterAngle;
import frc.robot.commands.shooter.ResetShooterAngle;
import frc.robot.commands.shooter.SetShooterAngle;
import frc.robot.commands.shooter.ShootSpeaker;
import frc.robot.subsystems.*;

public class RobotContainer {
  public static CommandXboxController robotController = new CommandXboxController(0); // joystick

  Shooter shooter = new Shooter();

  private SendableChooser<Command> autoChooser;
  private SendableChooser<Integer> pipeLineChooser = new SendableChooser<>();

  /** Configures the bindings for the robot's subsystems and commands. */
  private void configureBindings() {
    robotController.x().whileTrue(new ShootSpeaker(shooter, 0.82, 0.67));

    robotController.y().onTrue(new ResetShooterAngle(shooter));
    robotController.b().onTrue(new SetShooterAngle(shooter, 0));
    robotController.rightBumper().whileTrue(new AimShooterAngle(shooter, .2));
    robotController.leftBumper().whileTrue(new AimShooterAngle(shooter, -.4));
  }

  public RobotContainer() {
    // set up pipeline chooser
    pipeLineChooser.setDefaultOption("AprilTag", 0);
    pipeLineChooser.addOption("Note", 1);
    // pipeLineChooser.onChange((num) -> {drivetrain.getCamera().setPipeline(num);
    // System.out.println("pipeline set to " + num);});
    SmartDashboard.putData("Pipeline Chooser", pipeLineChooser);

    configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
