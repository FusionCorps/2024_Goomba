package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

  private TalonFX pivotMotor;
  private TalonFX rPivotMotor;
  private TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

  private DutyCycleEncoder pivotEncoder;

  public Pivot() {

    pivotEncoder = new DutyCycleEncoder(0);
    pivotMotor = new TalonFX(1);
    rPivotMotor = new TalonFX(5);

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

    pivotMotor.setPosition(pivotEncoder.get() * Constants.PIVOT_GEAR_RATIO);
  }

  public void setShooterAngle(double pct) {
    pivotMotor.set(pct);
    System.out.println(pivotMotor.getPosition());
  }

  public void resetShooterAngle() {
    pivotMotor.setPosition(0);
  }

  public void setAngle(double pos) {
    MotionMagicVoltage positionReq = new MotionMagicVoltage(0);
    pivotMotor.setControl(positionReq.withPosition(pos));
  }
}
