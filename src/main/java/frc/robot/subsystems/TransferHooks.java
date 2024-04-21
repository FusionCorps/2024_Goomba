package frc.robot.subsystems;

import static frc.robot.Constants.diagnosticsTab;
import static frc.robot.Constants.driverTab;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransferHookConstants;
import frc.robot.commands.TransferHooks.SetHooksPos;

// transfer hooks used in climb routine to hook onto chain
public class TransferHooks extends SubsystemBase {

  private TalonFX transferHookMotor;
  private TalonFXConfiguration transferHookConfiguration = new TalonFXConfiguration();
  double targetPosition;

  MotionMagicVoltage positionReq = new MotionMagicVoltage(0);

  public TransferHooks() {
    transferHookMotor = new TalonFX(TransferHookConstants.TRANSFER_HOOK_MOTOR_ID);

    transferHookConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    transferHookConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    transferHookConfiguration.Slot0.kV = TransferHookConstants.TRANSFER_HOOK_kV;
    transferHookConfiguration.Slot0.kP = TransferHookConstants.TRANSFER_HOOK_kP;
    transferHookConfiguration.Slot0.kI = TransferHookConstants.TRANSFER_HOOK_kI;
    transferHookConfiguration.Slot0.kD = TransferHookConstants.TRANSFER_HOOK_kD;

    transferHookConfiguration.MotionMagic.MotionMagicCruiseVelocity =
        TransferHookConstants.TRANSFER_HOOK_CRUISE_VELOCITY;
    transferHookConfiguration.MotionMagic.MotionMagicAcceleration =
        TransferHookConstants.TRANSFER_HOOK_ACCELERATION;
    transferHookConfiguration.MotionMagic.MotionMagicJerk =
        TransferHookConstants.TRANSFER_HOOK_JERK;

    transferHookMotor.getConfigurator().apply(transferHookConfiguration);

    transferHookMotor.setPosition(0);

    diagnosticsTab.addDouble(
        "TransferHooks Angle", () -> transferHookMotor.getPosition().getValueAsDouble());

    targetPosition = 0;

    driverTab
        .add(
            "Go to zero transfer hooks",
            new SetHooksPos(this, 0.0).alongWith(Commands.print(getName())))
        .withPosition(8, 0);
  }

  public double getPosition() {
    return transferHookMotor.getPosition().getValueAsDouble();
  }

  public void runHookPct(double pct) {
    transferHookMotor.set(pct);
  }

  public double getTargetPos() {
    return targetPosition;
  }

  public void setHookPos(double pos) {
    targetPosition = pos;
    transferHookMotor.setControl(positionReq.withPosition(pos));
  }

  public void holdAtTargetPos() {
    transferHookMotor.setControl(positionReq.withPosition(targetPosition));
  }

  public boolean reachedPos() {
    return Math.abs(targetPosition - transferHookMotor.getPosition().getValueAsDouble())
        < TransferHookConstants.TRANSFER_HOOK_ERROR;
  }
}
