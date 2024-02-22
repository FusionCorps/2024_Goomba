package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.SHOOTER_FREE_SPEED_LIMIT;
import static frc.robot.Constants.ShooterConstants.SHOOTER_LEFT_kD;
import static frc.robot.Constants.ShooterConstants.SHOOTER_LEFT_kFF;
import static frc.robot.Constants.ShooterConstants.SHOOTER_LEFT_kI;
import static frc.robot.Constants.ShooterConstants.SHOOTER_LEFT_kP;
import static frc.robot.Constants.ShooterConstants.SHOOTER_MOTOR_BOTTOM_ID;
import static frc.robot.Constants.ShooterConstants.SHOOTER_MOTOR_TOP_ID;
import static frc.robot.Constants.ShooterConstants.SHOOTER_RIGHT_kD;
import static frc.robot.Constants.ShooterConstants.SHOOTER_RIGHT_kFF;
import static frc.robot.Constants.ShooterConstants.SHOOTER_RIGHT_kI;
import static frc.robot.Constants.ShooterConstants.SHOOTER_RIGHT_kP;
import static frc.robot.Constants.ShooterConstants.SHOOTER_STALL_LIMIT_CURRENT;
import static frc.robot.Constants.diagnosticsTab;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

// Rev motor PID example:
// https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java

// Lookup table post:
// https://www.chiefdelphi.com/t/frc-4481-team-rembrandts-2024-build-thread-open-alliance/441907/324
// TODO: Use InterpolatingDoubleTreeMap for {distance: rpm} and {distance: pivotAngle} to determine
// optimal rpm
// CAVEAT: There are two motors moving at different velocities
/*
 * Order of testing:
 * 1) Determine optimal PID and FF constants for both shooter motors
 * 2) Test the effects of different RPMs on both motors on the note
 * 3) Make the lookup table and then test it
 */
public class Shooter extends SubsystemBase {

  private NetworkTable tuningTable =
      NetworkTableInstance.getDefault().getTable("Shooter PID Tuning");

  private CANSparkFlex leftMotor, rightMotor;
  private SparkPIDController leftController, rightController;

  public Shooter() {
    leftMotor = new CANSparkFlex(SHOOTER_MOTOR_BOTTOM_ID, MotorType.kBrushless);
    rightMotor = new CANSparkFlex(SHOOTER_MOTOR_TOP_ID, MotorType.kBrushless);
    rightMotor.setInverted(false);
    leftMotor.setInverted(true);

    // Set PID for left motor
    leftController = leftMotor.getPIDController();
    leftController.setP(SHOOTER_LEFT_kP);
    leftController.setI(SHOOTER_LEFT_kI);
    leftController.setD(SHOOTER_LEFT_kD);
    leftController.setFF(SHOOTER_LEFT_kFF);
    leftController.setIZone(0);
    leftController.setOutputRange(0, ShooterConstants.SHOOTER_MAX_RPM);
    leftMotor.setInverted(true);

    // // Set PID for right motor
    rightController = rightMotor.getPIDController();
    rightController.setP(SHOOTER_RIGHT_kP);
    rightController.setI(SHOOTER_RIGHT_kI);
    rightController.setD(SHOOTER_RIGHT_kD);
    rightController.setFF(SHOOTER_RIGHT_kFF);
    rightController.setIZone(0);
    rightController.setOutputRange(0, ShooterConstants.SHOOTER_MAX_RPM);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
    diagnosticsTab.addNumber("Shooter Vel Left", () -> leftMotor.getEncoder().getVelocity());
    diagnosticsTab.addNumber("Shooter Vel Right", () -> rightMotor.getEncoder().getVelocity());

    // tuningTable.getDoubleTopic("lSetpoint").publish();
    // tuningTable.getDoubleTopic("lkP").publish();
    // tuningTable.getDoubleTopic("lkI").publish();
    // tuningTable.getDoubleTopic("lkD").publish();
    // tuningTable.getDoubleTopic("lkFF").publish();

    // tuningTable.getDoubleTopic("rSetpoint").publish();
    // tuningTable.getDoubleTopic("rkP").publish();
    // tuningTable.getDoubleTopic("rkI").publish();
    // tuningTable.getDoubleTopic("rkD").publish();
    // tuningTable.getDoubleTopic("rkFF").publish();
  }

  // we can use this to tune the PID constants quickly
  public void periodic() {
    // rightController.setP(tuningTable.getEntry("rkP").getDouble(SHOOTER_RIGHT_kP));
    // rightController.setI(tuningTable.getEntry("rkI").getDouble(SHOOTER_RIGHT_kI));
    // rightController.setD(tuningTable.getEntry("rkD").getDouble(SHOOTER_RIGHT_kD));
    // rightController.setFF(tuningTable.getEntry("rkFF").getDouble(SHOOTER_RIGHT_kFF));

    // leftController.setP(tuningTable.getEntry("lkP").getDouble(SHOOTER_LEFT_kP));
    // leftController.setI(tuningTable.getEntry("lkI").getDouble(SHOOTER_LEFT_kI));
    // leftController.setD(tuningTable.getEntry("lkD").getDouble(SHOOTER_LEFT_kD));
    // leftController.setFF(tuningTable.getEntry("lkFF").getDouble(SHOOTER_LEFT_kFF));
  }

  /**
   * Runs the shooter motors using velocity PID control.
   *
   * @param leftRPM the RPM to set the left shooter to
   * @param rightRPM the RPM to set the right shooter to
   */
  public void shoot(double leftRPM, double rightRPM) {
    tuningTable.getEntry("lSetpoint").setDouble(leftRPM);
    tuningTable.getEntry("rSetpoint").setDouble(rightRPM);

    leftMotor.setSmartCurrentLimit(SHOOTER_STALL_LIMIT_CURRENT, SHOOTER_FREE_SPEED_LIMIT);
    rightMotor.setSmartCurrentLimit(SHOOTER_STALL_LIMIT_CURRENT, SHOOTER_FREE_SPEED_LIMIT);
    leftController.setReference(leftRPM, ControlType.kVelocity);
    rightController.setReference(rightRPM, ControlType.kVelocity);

    // System.out.println(
    //     rightMotor.getEncoder().getVelocity() + " " + leftMotor.getEncoder().getVelocity());
  }

  // returns whether both shooters have reached the target speed
  public boolean reachedSpeeds() {
    return true;
  }
}
