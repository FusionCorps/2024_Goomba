package frc.robot.subsystems;

import static frc.robot.Constants.CobraConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cobra extends SubsystemBase {
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

  public void setCobraPct(double pct) {
    cobraMotor.set(pct);
  }

  public void setCobraPos(double pos) {
    MotionMagicVoltage positionReq = new MotionMagicVoltage(0);
    cobraMotor.setControl(positionReq.withPosition(pos));
  }
}
