package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TransferHookConstants;

public class TransferHooks extends SubsystemBase {

    private TalonFX transferHookMotor;
    private TalonFXConfiguration transferHookConfiguration;
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

        transferHookConfiguration.MotionMagic.MotionMagicCruiseVelocity = TransferHookConstants.TRANSFER_HOOK_CRUISE_VELOCITY;
        transferHookConfiguration.MotionMagic.MotionMagicAcceleration = TransferHookConstants.TRANSFER_HOOK_ACCELERATION;
        transferHookConfiguration.MotionMagic.MotionMagicJerk = TransferHookConstants.TRANSFER_HOOK_JERK;

        transferHookMotor.getConfigurator().apply(transferHookConfiguration);

        transferHookMotor.setPosition(0);
    }

    public void runHookPct(double pct) {
        transferHookMotor.set(pct);
    }

    public void setHookPos(double pos) {
        targetPosition = pos;
        transferHookMotor.setControl(positionReq.withPosition(pos));
    }

    public boolean reachedPos() {
        return Math.abs(targetPosition
                - transferHookMotor.getPosition().getValueAsDouble()) < TransferHookConstants.TRANSFER_HOOK_ERROR;
    }

}
