// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.PIDConstants;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        // Set up limelight
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        limelightTable.getEntry("ledMode").setNumber(1); // turn off limelight LEDs
        limelightTable.getEntry("camMode").setNumber(0); // set limelight to vision processing mode
        limelightTable.getEntry("pipeline").setNumber(0); // set limelight to default pipeline
        limelightTable.getEntry("camerapose_robotspace_set").setDoubleArray(new double[5]);
        LimelightHelpers.setCameraPose_RobotSpace(
            Constants.LIMELIGHT_NAME, 
            0, 0, 0, 0, 0, 0);

        

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        System.out.println("Starting autonomous command: " + m_autonomousCommand.getName());

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        // System.out.println(RobotContainer.drivetrain.getPose());
    }


    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationPeriodic() {
    }
}
