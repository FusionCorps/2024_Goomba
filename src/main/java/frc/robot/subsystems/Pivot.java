package frc.robot.subsystems;

import static frc.robot.Constants.PivotConstants.PIVOT_ANGLES_MAP;
import static frc.robot.Constants.PivotConstants.PIVOT_STOW_POS;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    pivotMotor.getConfigurator().apply(pivotConfigs);
    pivotFollowerMotor.getConfigurator().apply(pivotConfigs);

    // pivotMotor.setControl(new Follower(pivotFollowerMotor.getDeviceID(), false));

    // sets the position of the motor acc. to through bore encoder once the encoder is ready
    new Trigger(pivotEncoder::isConnected)
        .onTrue(
            runOnce(
                () -> {
                  System.out.println("syncing pivot encoders init");
                  pivotMotor.setPosition(adjustedPivotEncoderAngle.getAsDouble());
                }));

    driverTab
        .addDouble("Pivot Angle", this::getPivotAngle)
        .withSize(2, 2)
        .withPosition(4, 0)
        .withWidget(BuiltInWidgets.kDial);

    diagnosticsTab.addDouble("Pivot Stow Pos", () -> PIVOT_STOW_POS);
    diagnosticsTab.addDouble("Pivot Motor Pos", () -> pivotMotor.getPosition().getValueAsDouble());
    diagnosticsTab.addDouble("Pivot Encoder Pos", () -> pivotEncoder.getAbsolutePosition());
    diagnosticsTab.addDouble(
        "Main Pivot Motor Voltage", () -> pivotMotor.getMotorVoltage().getValueAsDouble());
    diagnosticsTab.addDouble(
        "Follower Pivot Motor Voltage",
        () -> pivotFollowerMotor.getMotorVoltage().getValueAsDouble());
  }

  @Override
  public void periodic() {
    if (!motorConfigured
        && pivotEncoder.isConnected()
        && pivotMotor.getPosition().getValueAsDouble() != adjustedPivotEncoderAngle.getAsDouble()) {
      System.out.println("syncing pivot encoders periodic");

      pivotMotor.setPosition(adjustedPivotEncoderAngle.getAsDouble());
      pivotFollowerMotor.setPosition(adjustedPivotEncoderAngle.getAsDouble());
      motorConfigured = true;
    }
  }

  public void syncPosition() {
    pivotMotor.setPosition(adjustedPivotEncoderAngle.getAsDouble());
    pivotFollowerMotor.setPosition(adjustedPivotEncoderAngle.getAsDouble());
  }

  /**
   * Sets the pivot angle using a duty cycle percentage.
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

    // if(Math.abs(targetPos - pivotMotor.getPosition().getValueAsDouble()) >= errorThreshold){
    //   if(targetPos - pivotMotor.getPosition().getValueAsDouble() < 0){
    //     pivotMotor.set(-0.3);
    //     pivotFollowerMotor.set(-0.3);
    //   } else{
    //     pivotMotor.set(0.3);
    //     pivotFollowerMotor.set(0.3);
    //   }
    // } else{
    //   pivotMotor.set(0);
    //   pivotFollowerMotor.set(0);
    // }

    // System.out.println(pivotMotor.getMotorVoltage() + ", " +
    // pivotFollowerMotor.getMotorVoltage());
  }

  /** @return true if the pivot is close enough to its target position */
  public boolean reachedAngle() {
    return Math.abs(targetPos - pivotMotor.getPosition().getValue())
        < PivotConstants.PIVOT_ERROR_THRESHOLD;
  }

  public double getPivotAngle() {
    return pivotMotor.getPosition().getValueAsDouble();
  }
}
