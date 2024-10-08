// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.UtilFunctions;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
    // start logging for diagnostics
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), false);

    Shuffleboard.selectTab("Driver"); // default to driver tab

    SignalLogger.setPath("home/lvuser/logs");
    SignalLogger.start();

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    Constants.allianceColor = UtilFunctions.getAllianceColor();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println("Starting autonomous command: " + m_autonomousCommand.getName());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
}
