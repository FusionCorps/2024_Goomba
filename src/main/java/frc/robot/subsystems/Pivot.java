package frc.robot.subsystems;

import static frc.robot.Constants.PivotConstants.PIVOT_ANGLES_MAP;
import static frc.robot.Constants.PivotConstants.PIVOT_ARM_INIT_POSE;
import static frc.robot.Constants.PivotConstants.PIVOT_STOW_POS;
import static frc.robot.Constants.PivotConstants.PIVOT_SUB_POS;
import static frc.robot.Constants.diagnosticsTab;
import static frc.robot.Constants.driverTab;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  private TalonFX pivotMotor, pivotFollowerMotor;
  private TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
  double errorThreshold = 0.5;

  boolean motorConfigured = false;

  // the target position of the pivot
  private double targetPos;

  MotionMagicVoltage positionReq = new MotionMagicVoltage(0);

  public Pivot() {
    PIVOT_ANGLES_MAP.put(1.08, PIVOT_SUB_POS);
    PIVOT_ANGLES_MAP.put(1.43, 21.68115234375);
    PIVOT_ANGLES_MAP.put(1.84, 17.3193359375);
    PIVOT_ANGLES_MAP.put(2.39, 13.03662109375);
    PIVOT_ANGLES_MAP.put(2.82, 11.22265625);
    PIVOT_ANGLES_MAP.put(3.30, 9.7099609375);
    PIVOT_ANGLES_MAP.put(3.46, 9.29541015625);
    PIVOT_ANGLES_MAP.put(4.00, 8.2);

    pivotMotor = new TalonFX(PivotConstants.PIVOT_MOTOR_ID);
    pivotFollowerMotor = new TalonFX(PivotConstants.PIVOT_FOLLOWER_MOTOR_ID);

    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotConfigs.Slot0.kV = PivotConstants.PIVOT_kV;
    pivotConfigs.Slot0.kP = PivotConstants.PIVOT_kP;
    pivotConfigs.Slot0.kI = PivotConstants.PIVOT_kI;
    pivotConfigs.Slot0.kD = PivotConstants.PIVOT_kD;

    pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.PIVOT_CRUISE_VELOCITY;
    pivotConfigs.MotionMagic.MotionMagicAcceleration = PivotConstants.PIVOT_ACCELERATION;
    pivotConfigs.MotionMagic.MotionMagicJerk = PivotConstants.PIVOT_JERK;

    // TODO: Set the soft limits for the pivot motor
    // pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    // pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    pivotMotor.getConfigurator().apply(pivotConfigs);
    pivotFollowerMotor.getConfigurator().apply(pivotConfigs);

    pivotMotor.setPosition(PIVOT_ARM_INIT_POSE);
    pivotFollowerMotor.setPosition(PIVOT_ARM_INIT_POSE);

    driverTab
        .addDouble("Pivot Angle", this::getPivotAngle)
        .withSize(2, 2)
        .withPosition(4, 0)
        .withWidget(BuiltInWidgets.kTextView);

    diagnosticsTab.addDouble("Pivot Stow Pos", () -> PIVOT_STOW_POS);
    diagnosticsTab.addDouble("Pivot Motor Pos", () -> pivotMotor.getPosition().getValueAsDouble());
    diagnosticsTab.addDouble(
        "Main Pivot Motor Voltage", () -> pivotMotor.getMotorVoltage().getValueAsDouble());
    diagnosticsTab.addDouble(
        "Follower Pivot Motor Voltage",
        () -> pivotFollowerMotor.getMotorVoltage().getValueAsDouble());
  }

  @Override
  public void periodic() {}

  /**
   * Sets the pivot angle using a duty cycle percentage.
   *
   * @param pct the percentage to set the shooter to
   */
  public void setPivotPct(double pct) {

    pivotMotor.set(pct);
    pivotFollowerMotor.set(pct);

    targetPos = pivotMotor.getPosition().getValueAsDouble();

  }

  /**
   * Sets targetpos to current pivot angle
   */

   public void setTargetPos(){

    targetPos = pivotMotor.getPosition().getValueAsDouble();
   }

  /** Zeroes the pivot angle at the current angle. */
  public void resetPivotAngle() {
    pivotMotor.setPosition(0);
    pivotFollowerMotor.setPosition(0);
  }

  /**
   * Sets the pivot angle using Motion Magic.
   *
   * @param pos the position to set the shooter to, in rotations
   */
  public void setPivotAngle(double pos) {
    targetPos = pos;

    pivotMotor.setControl(positionReq.withPosition(targetPos));
    pivotFollowerMotor.setControl(positionReq.withPosition(targetPos));
  }

  public void setPivotAngle(double pos, double vel, double accel, double jerk) {
    targetPos = pos;

    pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = vel;
    pivotConfigs.MotionMagic.MotionMagicAcceleration = accel;
    pivotConfigs.MotionMagic.MotionMagicJerk = jerk;

    pivotMotor.getConfigurator().apply(pivotConfigs);
    pivotFollowerMotor.getConfigurator().apply(pivotConfigs);

    pivotMotor.setControl(positionReq.withPosition(targetPos));
    pivotFollowerMotor.setControl(positionReq.withPosition(targetPos));
  }

  /**
   * @return true if the pivot is close enough to its target position
   */
  public boolean reachedAngle() {
    return Math.abs(targetPos - pivotMotor.getPosition().getValue())
        < PivotConstants.PIVOT_ERROR_THRESHOLD;
  }

  public double getPivotAngle() {
    return pivotMotor.getPosition().getValueAsDouble();
  }

  public void holdPivotPos() {
    pivotMotor.setControl(positionReq.withPosition(targetPos));
    pivotFollowerMotor.setControl(positionReq.withPosition(targetPos));
  }
}
