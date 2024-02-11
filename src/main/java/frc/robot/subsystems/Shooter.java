package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.SHOOTER_MOTOR_BOTTOM_ID;
import static frc.robot.Constants.ShooterConstants.SHOOTER_MOTOR_TOP_ID;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkFlex leftMotor, rightMotor;
  private SparkPIDController leftController, rightController;

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
    rightController.setI(0);
    rightController.setD(0);
    rightController.setFF(0);
    rightController.setIZone(0);
    rightController.setOutputRange(-1, 1);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  public void periodic() {}

  
  /**
   * Runs the shooter motors using duty cycle percentages.
   *
   * @param leftPct the percentage to set the left shooter to
   * @param rightPct the percentage to set the right shooter to
   */
  public void shoot(double leftPct, double rightPct) {
    // leftController.setReference(leftRpm, ControlType.kVelocity);
    // rightController.setReference(-rightRpm, ControlType.kVelocity);

    // TODO: check if this is correct based on setInverted
    // rightController.setReference(rightRpm, ControlType.kVelocity);

    leftMotor.setSmartCurrentLimit(60, 5500);
    rightMotor.setSmartCurrentLimit(60, 5500);
    rightMotor.set(rightPct);
    leftMotor.set(leftPct);
    System.out.println(
        rightMotor.getEncoder().getVelocity() + " " + leftMotor.getEncoder().getVelocity());
  }

  //TODO: Find out how to set rpm of shooter
  // returns whether both shooters have reached the target speed
  public boolean reachedSpeeds() {
    return true;
  }

  // public void setShooterAngle(double angleOfShooter) {
  //     pivotMotor.setPosition(angleOfShooter);
  // }

  // public Command aimShooterAngleCommand(double angle) {
  //     return run(() -> setShooterAngle(angle));
  // }

  // public Command shootSpeakerCommand(double leftRPM, double rightRPM) {
  //   return run(() -> shoot(leftRPM, rightRPM)).finallyDo(() -> shoot(0, 0));
  // }

}
