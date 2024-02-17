package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.SHOOTER_MAX_RPM;
import static frc.robot.Constants.ShooterConstants.SHOOTER_MOTOR_BOTTOM_ID;
import static frc.robot.Constants.ShooterConstants.SHOOTER_MOTOR_TOP_ID;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  private NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("flywheelTuning");

  private CANSparkFlex leftMotor, rightMotor;
  private SparkPIDController leftController, rightController;

  // the setpoint (rpm) of the right motor
  private double rSetpoint = 0;

  public Shooter() {
    leftMotor = new CANSparkFlex(SHOOTER_MOTOR_BOTTOM_ID, MotorType.kBrushless);
    rightMotor = new CANSparkFlex(SHOOTER_MOTOR_TOP_ID, MotorType.kBrushless);
    rightMotor.setInverted(false);
    leftMotor.setInverted(true);

    // Set PID for left motor
    leftController = leftMotor.getPIDController();
    leftController.setP(0);
    leftController.setI(0.0);
    leftController.setD(0.0);
    leftController.setFF(0);
    leftController.setIZone(0);
    leftController.setOutputRange(-1, 1);
    leftMotor.setInverted(true);

    // // Set PID for right motor
    rightController = rightMotor.getPIDController();
    rightController.setP(0);
    // rightController.setP(0.00006);
    rightController.setI(0);
    rightController.setD(0);
    rightController.setFF(0);
    // rightController.setFF(0.000015);
    rightController.setIZone(0);
    rightController.setOutputRange(-SHOOTER_MAX_RPM, SHOOTER_MAX_RPM);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  // method that sets the rpm of the right motor
  public void shootRightRPM(double rpm) {
    rSetpoint = rpm;
    rightController.setReference(rpm, ControlType.kVelocity);
  }

  // updates the reference and the output in NT for PID tuning
  // (drag&drop these entries into an AdvantageScope graph)
  public void periodic() {
    tuningTable.getEntry("setpoint").setDouble(rSetpoint);
    tuningTable.getEntry("output").setDouble(rightMotor.getEncoder().getVelocity());
  }

  /**
   * Runs the shooter motors using duty cycle percentages.
   *
   * @param leftPct the percentage to set the left shooter to
   * @param rightPct the percentage to set the right shooter to
   */
  public void shoot(double leftPct, double rightPct) {
    leftMotor.setSmartCurrentLimit(60, 5500);
    rightMotor.setSmartCurrentLimit(60, 5500);
    rightMotor.set(rightPct);
    leftMotor.set(leftPct);
    //System.out.println(
       // rightMotor.getEncoder().getVelocity() + " " + leftMotor.getEncoder().getVelocity());
  }

  // returns whether both shooters have reached the target speed
  public boolean reachedSpeeds() {
    return true;
  }

  public void setVelocities(double leftRPM, double rightRPM) {
    leftController.setReference(leftRPM, ControlType.kVelocity);
    rightController.setReference(rightRPM, ControlType.kVelocity);
  }
}
