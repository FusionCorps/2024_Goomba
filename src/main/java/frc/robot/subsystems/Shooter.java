package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.SHOOTER_FREE_SPEED_LIMIT;
import static frc.robot.Constants.ShooterConstants.SHOOTER_LEFT_kD;
import static frc.robot.Constants.ShooterConstants.SHOOTER_LEFT_kFF;
import static frc.robot.Constants.ShooterConstants.SHOOTER_LEFT_kI;
import static frc.robot.Constants.ShooterConstants.SHOOTER_LEFT_kP;
import static frc.robot.Constants.ShooterConstants.SHOOTER_MAX_RPM;
import static frc.robot.Constants.ShooterConstants.SHOOTER_MOTOR_BOTTOM_ID;
import static frc.robot.Constants.ShooterConstants.SHOOTER_MOTOR_TOP_ID;
import static frc.robot.Constants.ShooterConstants.SHOOTER_RIGHT_kD;
import static frc.robot.Constants.ShooterConstants.SHOOTER_RIGHT_kFF;
import static frc.robot.Constants.ShooterConstants.SHOOTER_RIGHT_kI;
import static frc.robot.Constants.ShooterConstants.SHOOTER_RIGHT_kP;
import static frc.robot.Constants.ShooterConstants.SHOOTER_STALL_LIMIT_CURRENT;
import static frc.robot.Constants.diagnosticsTab;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private NetworkTable tuningTable =
      NetworkTableInstance.getDefault().getTable("Shooter PID Tuning");

  private CANSparkFlex leftMotor, rightMotor;
  private SparkPIDController leftController, rightController;
  double leftRPM, rightRPM;

  ShuffleboardTab tab = Shuffleboard.getTab("General");

  public GenericEntry isOuttaking =
      tab.add("Shoot Out", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

  public GenericEntry isShooting =  tab.add("Shoot In", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

  public Shooter() {

    isOuttaking.setBoolean(false);
    isShooting.setBoolean(false);
    
    leftMotor = new CANSparkFlex(SHOOTER_MOTOR_BOTTOM_ID, MotorType.kBrushless);
    rightMotor = new CANSparkFlex(SHOOTER_MOTOR_TOP_ID, MotorType.kBrushless);
    rightMotor.setInverted(false);
    leftMotor.setInverted(true);
    rightMotor.setIdleMode(IdleMode.kCoast);
    leftMotor.setIdleMode(IdleMode.kCoast);

    // Set PID for left motor
    leftController = leftMotor.getPIDController();
    leftController.setP(SHOOTER_LEFT_kP);
    leftController.setI(SHOOTER_LEFT_kI);
    leftController.setD(SHOOTER_LEFT_kD);
    leftController.setFF(SHOOTER_LEFT_kFF);
    leftController.setIZone(0);
    leftController.setOutputRange(-SHOOTER_MAX_RPM, ShooterConstants.SHOOTER_MAX_RPM);
    leftMotor.setInverted(true);

    // // Set PID for right motor
    rightController = rightMotor.getPIDController();
    rightController.setP(SHOOTER_RIGHT_kP);
    rightController.setI(SHOOTER_RIGHT_kI);
    rightController.setD(SHOOTER_RIGHT_kD);
    rightController.setFF(SHOOTER_RIGHT_kFF);
    rightController.setIZone(0);
    rightController.setOutputRange(-SHOOTER_MAX_RPM, ShooterConstants.SHOOTER_MAX_RPM);

    leftMotor.setSmartCurrentLimit(SHOOTER_STALL_LIMIT_CURRENT, SHOOTER_FREE_SPEED_LIMIT);
    rightMotor.setSmartCurrentLimit(SHOOTER_STALL_LIMIT_CURRENT, SHOOTER_FREE_SPEED_LIMIT);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
    diagnosticsTab.addNumber("Shooter Vel Left", () -> leftMotor.getEncoder().getVelocity());
    diagnosticsTab.addNumber("Shooter Vel Right", () -> rightMotor.getEncoder().getVelocity());
  }

  /**
   * Runs the shooter motors using velocity PID control.
   *
   * @param leftRPM the RPM to set the left shooter to
   * @param rightRPM the RPM to set the right shooter to
   */
  public void setRPMs(double inleftRPM, double inrightRPM) {
    leftRPM = inleftRPM;
    rightRPM = inrightRPM;

    tuningTable.getEntry("lSetpoint").setDouble(leftRPM);
    tuningTable.getEntry("rSetpoint").setDouble(rightRPM);

    leftController.setReference(leftRPM, ControlType.kVelocity);
    rightController.setReference(rightRPM, ControlType.kVelocity);
  }

  // returns whether both shooters have reached the target speed
  public boolean reachedSpeeds() {
    return Math.abs(leftMotor.getEncoder().getVelocity() - leftRPM) < 100.0
        && Math.abs(rightMotor.getEncoder().getVelocity() - rightRPM) < 100.0;
  }
}
