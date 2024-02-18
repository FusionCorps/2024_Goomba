package frc.robot.subsystems;

import static frc.robot.Constants.driverTab;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {

  private TalonFX pivotMotor, pivotFollowerMotor;
  private TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

  PositionDutyCycle stabilizingDutyCycle = new PositionDutyCycle(0);

  // the target position of the pivot
  private double targetPos;

  private DutyCycleEncoder pivotEncoder; // through bore encoder

  public Pivot() {
    pivotEncoder = new DutyCycleEncoder(0);
    pivotMotor = new TalonFX(PivotConstants.PIVOT_MOTOR_ID);
    pivotFollowerMotor = new TalonFX(PivotConstants.PIVOT_FOLLOWER_MOTOR_ID);

    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotConfigs.Slot0.kV = 0;
    pivotConfigs.Slot0.kP = PivotConstants.PIVOT_kP;
    pivotConfigs.Slot0.kI = PivotConstants.PIVOT_kI;
    pivotConfigs.Slot0.kD = PivotConstants.PIVOT_kD;

    pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = 1000;
    pivotConfigs.MotionMagic.MotionMagicAcceleration = 400;
    pivotConfigs.MotionMagic.MotionMagicJerk = 350;

    pivotMotor.getConfigurator().apply(pivotConfigs);
    pivotFollowerMotor.getConfigurator().apply(pivotConfigs);

    // pivotMotor.setControl(new Follower(pivotFollowerMotor.getDeviceID(), false));

    // sets the position of the motor acc. to through bore encoder once the encoder is ready
    new Trigger(pivotEncoder::isConnected)
        .onTrue(
            runOnce(
                () -> {
                  System.out.println("here");
                  pivotMotor.setPosition(
                      (pivotEncoder.getAbsolutePosition()) * PivotConstants.PIVOT_GEAR_RATIO);
                }));

    driverTab
        .addDouble("Pivot Angle", this::getPivotAngle)
        .withSize(2, 2)
        .withPosition(4, 0)
        .withWidget(BuiltInWidgets.kDial);
  }

  @Override
  public void periodic() {
    System.out.println(
        PivotConstants.PIVOT_OFFSET * PivotConstants.PIVOT_GEAR_RATIO
            + ", "
            + pivotMotor.getPosition().getValueAsDouble()
            + " , "
            + pivotEncoder.getAbsolutePosition() * PivotConstants.PIVOT_GEAR_RATIO);

    // if(!motorConfigured && pivotEncoder.isConnected() &&
    // pivotMotor.getPosition().getValueAsDouble()/PivotConstants.PIVOT_GEAR_RATIO !=
    // pivotEncoder.getAbsolutePosition()){
    //
    //   motorConfigured = true;
    // }

    pivotMotor.setPosition(pivotEncoder.getAbsolutePosition() * PivotConstants.PIVOT_GEAR_RATIO);
    pivotFollowerMotor.setPosition(
        pivotEncoder.getAbsolutePosition() * PivotConstants.PIVOT_GEAR_RATIO);
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
    pivotFollowerMotor.set(pct);
    System.out.println(pivotMotor.getMotorVoltage() + ", " + pivotFollowerMotor.getMotorVoltage());
    // System.out.println(pivotMotor.getPosition() + ", " + pivotEncoder.getAbsolutePosition());

  }

  /** Zeroes the pivot angle to the current angle. */
  public void resetPivotAngle() {
    pivotMotor.setPosition(0);
    pivotFollowerMotor.setPosition(0);
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
    pivotMotor.setControl(positionReq.withPosition(targetPos));
    pivotFollowerMotor.setControl(positionReq.withPosition(targetPos));
  }

  // whether the pivot has reached the setpoint
  public boolean reachedAngle() {
    return Math.abs(targetPos - pivotMotor.getPosition().getValue())
        < PivotConstants.PIVOT_ERROR_THRESHOLD;
  }

  public double getPivotAngle() {
    return pivotMotor.getPosition().getValueAsDouble();
  }

  // public void stabilizeMotors(){
  //   pivotMotor.setControl(stabilizingDutyCycle.withPosition(getPivotAngle()));
  // }
}
