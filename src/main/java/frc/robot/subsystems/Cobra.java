package frc.robot.subsystems;

import static frc.robot.Constants.CobraConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;

public class Cobra extends SubsystemBase {
  @AutoLog
  public static class CobraInputs {
    public double motorPosRot = 0.0;
    public double motorVoltage = 0.0;
    public double motorCurrent = 0.0;
  }

  private CobraInputsAutoLogged inputs = new CobraInputsAutoLogged();

  TalonFX cobraMotor;
  TalonFXConfiguration cobraConfigs = new TalonFXConfiguration();

  public Cobra() {
    cobraConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cobraMotor = new TalonFX(cbrMotorID);
    cobraConfigs.Slot0.kV = 0;
    cobraConfigs.Slot0.kP = cbrKP;
    cobraConfigs.Slot0.kI = cbrKI;
    cobraConfigs.Slot0.kD = cbrKD;

    cobraConfigs.MotionMagic.MotionMagicCruiseVelocity = 40;
    cobraConfigs.MotionMagic.MotionMagicAcceleration = 15;
    cobraConfigs.MotionMagic.MotionMagicJerk = 10;
    cobraMotor.getConfigurator().apply(cobraConfigs);
  }

  @Override
  public void periodic() {
    inputs.motorPosRot = cobraMotor.getPosition().getValueAsDouble();
    inputs.motorVoltage = cobraMotor.getMotorVoltage().getValueAsDouble();
    inputs.motorCurrent = cobraMotor.getStatorCurrent().getValueAsDouble();
  }

  /**
   * Sets the cobra angle using a duty cycle percentage.
   *
   * @param pct the percentage to set the cobra to
   */
  public void setCobraPct(double pct) {
    cobraMotor.set(pct);
  }

  /**
   * Sets the cobra angle using Motion Magic.
   *
   * @param pos the position to set the cobra to, in rotations
   */
  public void setCobraPos(double pos) {
    MotionMagicVoltage positionReq = new MotionMagicVoltage(0);
    cobraMotor.setControl(positionReq.withPosition(pos));
  }
}
