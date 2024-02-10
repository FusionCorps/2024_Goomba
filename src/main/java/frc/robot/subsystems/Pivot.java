package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {

  private TalonFX pivotMotor, pivotFollowerMotor;
  private TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

  private DutyCycleEncoder pivotEncoder; // through bore encoder

  public Pivot() {
    pivotEncoder = new DutyCycleEncoder(0);
    pivotMotor = new TalonFX(PivotConstants.PIVOT_MOTOR_ID);
    pivotFollowerMotor = new TalonFX(PivotConstants.PIVOT_FOLLOWER_MOTOR_ID);

    pivotMotor.setPosition(pivotEncoder.getDistance() / PivotConstants.PIVOT_GEAR_RATIO);

    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfigs.Slot0.kV = 0;
    pivotConfigs.Slot0.kP = PivotConstants.PIVOT_kP;
    pivotConfigs.Slot0.kI = PivotConstants.PIVOT_kI;
    pivotConfigs.Slot0.kD = PivotConstants.PIVOT_kD;

    pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = 40;
    pivotConfigs.MotionMagic.MotionMagicAcceleration = 15;
    pivotConfigs.MotionMagic.MotionMagicJerk = 10;

    pivotMotor.getConfigurator().apply(pivotConfigs);

    pivotMotor.setControl(new Follower(pivotFollowerMotor.getDeviceID(), true));
  }

  @Override
  public void periodic() {
    // pivotMotor.setPosition(pivotEncoder.getDistance() / PivotConstants.PIVOT_GEAR_RATIO);
  }

  /**
   * Sets the pivot angle using a duty cycle percentage.
   *
   * @param pct the percentage to set the shooter to
   */
  public void setPivotAngle(double pct) {
    pivotMotor.setPosition(pivotEncoder.getDistance() / PivotConstants.PIVOT_GEAR_RATIO);
    pivotMotor.set(pct);
    System.out.println(pivotMotor.getPosition() + ", " + pivotEncoder.getDistance());
  }

  /** Zeroes the pivot angle to the current angle. */
  public void resetPivotAngle() {
    pivotMotor.setPosition(0);
  }

  /**
   * Sets the pivot angle using Motion Magic.
   *
   * @param pos the position to set the shooter to, in rotations
   */
  public void setAngle(double pos) {
    pivotMotor.setPosition(pivotEncoder.getDistance() / PivotConstants.PIVOT_GEAR_RATIO);
    MotionMagicVoltage positionReq = new MotionMagicVoltage(0);
    pivotMotor.setControl(positionReq.withPosition(pos));
  }
}
