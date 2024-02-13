package frc.robot.subsystems;

import static frc.robot.Constants.driverTab;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {

  private TalonFX pivotMotor, pivotFollowerMotor;
  private TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

  // the target position of the pivot
  private double targetPos;

  private DutyCycleEncoder pivotEncoder; // through bore encoder

  public Pivot() {
    pivotEncoder = new DutyCycleEncoder(0);
    pivotMotor = new TalonFX(PivotConstants.PIVOT_MOTOR_ID);
    pivotFollowerMotor = new TalonFX(PivotConstants.PIVOT_FOLLOWER_MOTOR_ID);

    // sets the position of the motor acc. to through bore encoder once the encoder is ready
    new Trigger(pivotEncoder::isConnected)
        .onTrue(
            runOnce(
                () -> {
                  pivotMotor.setPosition(
                      pivotEncoder.getAbsolutePosition() * PivotConstants.PIVOT_GEAR_RATIO);
                }));

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

    driverTab.addDouble("Pivot Angle", this::getPivotAngle);
  }

  @Override
  public void periodic() {
    System.out.println(pivotMotor.getPosition() + ", " + pivotEncoder.getAbsolutePosition());
    // pivotMotor.setPosition(pivotEncoder.getDistance() / PivotConstants.PIVOT_GEAR_RATIO);
  }

  /**
   * Sets the pivot angle using a duty cycle percentage.
   *
   * @param pct the percentage to set the shooter to
   */
  public void setPivotPct(double pct) {
    // pivotMotor.setPosition(pivotEncoder.getDistance() * PivotConstants.PIVOT_GEAR_RATIO);
    pivotMotor.set(pct);
    
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
    targetPos = pos;
    // pivotMotor.setPosition(pivotEncoder.getDistance() * PivotConstants.PIVOT_GEAR_RATIO);
    MotionMagicVoltage positionReq = new MotionMagicVoltage(0);
    pivotMotor.setControl(positionReq.withPosition(pos));
  }

  // whether the pivot has reached the setpoint
  public boolean reachedAngle() {
    return Math.abs(targetPos - pivotMotor.getPosition().getValue())
        < PivotConstants.PIVOT_ERROR_THRESHOLD;
  }

  public double getPivotAngle() {
    return pivotMotor.getPosition().getValue();
  }
}
