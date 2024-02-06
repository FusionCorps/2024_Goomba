package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkFlex leftMotor, rightMotor;
  private SparkPIDController leftController, rightController;

  private TalonFX pivotMotor;
  private TalonFX rPivotMotor;
  private TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
  private TalonFXConfiguration rPivotConfigs = new TalonFXConfiguration();

  public Shooter() {
    leftMotor = new CANSparkFlex(Constants.BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new CANSparkFlex(Constants.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    rightMotor.setInverted(true);

    pivotMotor = new TalonFX(1);
    rPivotMotor = new TalonFX(2);

    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfigs.Slot0.kV = 0;
    pivotConfigs.Slot0.kP = Constants.PIVOT_kP;
    pivotConfigs.Slot0.kI = Constants.PIVOT_kI;
    pivotConfigs.Slot0.kD = Constants.PIVOT_kD;

    pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = 40;
    pivotConfigs.MotionMagic.MotionMagicAcceleration = 15;
    pivotConfigs.MotionMagic.MotionMagicJerk = 10;

    pivotMotor.getConfigurator().apply(pivotConfigs);

  
    pivotMotor.setControl(new Follower(rPivotMotor.getDeviceID(), true));
    

  

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

  public void shoot(double leftRpm, double rightRpm) {
    // leftController.setReference(leftRpm, ControlType.kVelocity);
    // rightController.setReference(-rightRpm, ControlType.kVelocity);

    // TODO: check if this is correct based on setInverted
    // rightController.setReference(rightRpm, ControlType.kVelocity);

    leftMotor.setSmartCurrentLimit(60, 5500);
    rightMotor.setSmartCurrentLimit(60, 5500);
    rightMotor.set(rightRpm);
    leftMotor.set(leftRpm);
    // System.out.println(
    //     rightMotor.getEncoder().getVelocity() + " " + leftMotor.getEncoder().getVelocity());
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

  public void setShooterAngle(double pct) {
      pivotMotor.set(pct);
      System.out.println(pivotMotor.getPosition());
  }

  public void resetShooterAngle(){
    pivotMotor.setPosition(0);
  }
}
