package frc.robot.subsystems;

import static frc.robot.Constants.PivotConstants.PIVOT_ANGLES_MAP;
import static frc.robot.Constants.PivotConstants.PIVOT_ARM_INIT_POSE;
import static frc.robot.Constants.PivotConstants.PIVOT_GEAR_RATIO;
import static frc.robot.Constants.PivotConstants.PIVOT_STOW_POS;
import static frc.robot.Constants.PivotConstants.PIVOT_SUB_POS;
import static frc.robot.Constants.diagnosticsTab;
import static frc.robot.Constants.driverTab;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import java.util.function.DoubleSupplier;

public class Pivot extends SubsystemBase {
  private TalonFX pivotMotor, pivotFollowerMotor;
  private TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
  double errorThreshold = 0.5;

  boolean motorConfigured = false;

  // the target position of the pivot
  private double targetPos;

  private DutyCycleEncoder pivotEncoder; // through bore encoder
  private DoubleSupplier adjustedPivotEncoderAngle;

  MotionMagicVoltage positionReq = new MotionMagicVoltage(0);

  public Pivot() {
    PIVOT_ANGLES_MAP.put(0.99, PIVOT_SUB_POS);
    PIVOT_ANGLES_MAP.put(1.48, 23.2041015625);
    PIVOT_ANGLES_MAP.put(1.85, 16.81201171875);
    PIVOT_ANGLES_MAP.put(2.25, 13.64697265625);
    PIVOT_ANGLES_MAP.put(2.5, 12.51416015625);
    PIVOT_ANGLES_MAP.put(2.63, 10.0927734375);
    PIVOT_ANGLES_MAP.put(3.17, 9.45263671875);
    PIVOT_ANGLES_MAP.put(3.5, 8.953125);

    pivotEncoder = new DutyCycleEncoder(2);
    adjustedPivotEncoderAngle = () -> pivotEncoder.getAbsolutePosition() * PIVOT_GEAR_RATIO;

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
    diagnosticsTab.addDouble("Pivot Encoder Pos", () -> adjustedPivotEncoderAngle.getAsDouble());
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
