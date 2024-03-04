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

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

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
  @AutoLog
  public static class ShooterInputs {
    public double leftMotorVelocityRot = 0.0;
    public double rightMotorVelocityRot = 0.0;
    public double leftMotorVoltage = 0.0;
    public double leftMotorCurrent = 0.0;
    public double rightMotorVoltage = 0.0;
    public double rightMotorCurrent = 0.0;
  }

  public ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

  private NetworkTable tuningTable =
      NetworkTableInstance.getDefault().getTable("Shooter PID Tuning");

  private CANSparkFlex leftMotor, rightMotor;
  private SparkPIDController leftController, rightController;
  double leftRPM, rightRPM;

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

    leftMotor.setSmartCurrentLimit(SHOOTER_STALL_LIMIT_CURRENT, SHOOTER_FREE_SPEED_LIMIT);
    rightMotor.setSmartCurrentLimit(SHOOTER_STALL_LIMIT_CURRENT, SHOOTER_FREE_SPEED_LIMIT);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  @Override
  public void periodic() {
    inputs.leftMotorVelocityRot = leftMotor.getEncoder().getVelocity();
    inputs.rightMotorVelocityRot = rightMotor.getEncoder().getVelocity();
    inputs.leftMotorVoltage = leftMotor.getBusVoltage();
    inputs.leftMotorCurrent = leftMotor.getOutputCurrent();
    inputs.rightMotorVoltage = rightMotor.getBusVoltage();
    inputs.rightMotorCurrent = rightMotor.getOutputCurrent();
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
  @AutoLogOutput
  public boolean reachedSpeeds() {
    return Math.abs(leftMotor.getEncoder().getVelocity() - leftRPM) < 50.0
        && Math.abs(rightMotor.getEncoder().getVelocity() - rightRPM) < 50.0;
  }
}
