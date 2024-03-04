package frc.robot.subsystems;

import static frc.robot.Constants.PivotConstants.PIVOT_ANGLES_MAP;
import static frc.robot.Constants.PivotConstants.PIVOT_STOW_POS;
import static frc.robot.Constants.driverTab;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PivotConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public class Pivot extends SubsystemBase {
  @AutoLog
  public static class PivotInputs {
    public double motorPosRot = 0.0;
    public double encoderPosRot = 0.0;
    public double mainMotorVoltage = 0.0;
    public double mainMotorCurrent = 0.0;
    public double followerMotorVoltage = 0.0;
    public double followerMotorCurrent = 0.0;
  }

  PivotInputsAutoLogged inputs = new PivotInputsAutoLogged();

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
    PIVOT_ANGLES_MAP.put(1.87, 66.22);
    PIVOT_ANGLES_MAP.put(2.27, 69.23);
    PIVOT_ANGLES_MAP.put(2.67, PIVOT_STOW_POS);
    PIVOT_ANGLES_MAP.put(3.06, 72.02);
    PIVOT_ANGLES_MAP.put(3.73, 74.07);
    PIVOT_ANGLES_MAP.put(3.37, 72.2);
    pivotEncoder = new DutyCycleEncoder(0);
    adjustedPivotEncoderAngle =
        () -> pivotEncoder.getAbsolutePosition() * PivotConstants.PIVOT_GEAR_RATIO;

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

    // pivotMotor.setControl(new Follower(pivotFollowerMotor.getDeviceID(), false));

    // sets the position of the motor acc. to through bore encoder once the encoder is ready
    new Trigger(pivotEncoder::isConnected)
        .onTrue(
            run(() -> {
                  System.out.println("syncing pivot encoders");
                  syncPosition();
                })
                .until(
                    () ->
                        pivotMotor.getPosition().getValueAsDouble()
                                - adjustedPivotEncoderAngle.getAsDouble()
                            < 0.1));

    driverTab
        .addDouble("Pivot Angle", this::getPivotAngle)
        .withSize(2, 2)
        .withPosition(4, 0)
        .withWidget(BuiltInWidgets.kDial);
  }

  @Override
  public void periodic() {
    inputs.motorPosRot = pivotMotor.getPosition().getValueAsDouble();
    inputs.encoderPosRot = adjustedPivotEncoderAngle.getAsDouble();
    inputs.mainMotorVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.mainMotorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
    inputs.followerMotorVoltage = pivotFollowerMotor.getMotorVoltage().getValueAsDouble();
    inputs.followerMotorCurrent = pivotFollowerMotor.getStatorCurrent().getValueAsDouble();
  }

  public void syncPosition() {
    pivotMotor.setPosition(adjustedPivotEncoderAngle.getAsDouble());
    pivotFollowerMotor.setPosition(adjustedPivotEncoderAngle.getAsDouble());
  }

  /**
   * Sets the pivot angle using a duty cycle percentage.
   *
   * @param pct the percentage to set the shooter to
   */
  public void setPivotPct(double pct) {
    // pivotMotor.setPosition(pivotEncoder.getDistance() * PivotConstants.PIVOT_GEAR_RATIO);
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
  @AutoLogOutput
  public boolean reachedAngle() {
    return Math.abs(targetPos - pivotMotor.getPosition().getValue())
        < PivotConstants.PIVOT_ERROR_THRESHOLD;
  }

  @AutoLogOutput
  public double getPivotAngle() {
    return pivotMotor.getPosition().getValueAsDouble();
  }

  @AutoLogOutput
  public double getPivotEncoderAngle() {
    return adjustedPivotEncoderAngle.getAsDouble();
  }
}
