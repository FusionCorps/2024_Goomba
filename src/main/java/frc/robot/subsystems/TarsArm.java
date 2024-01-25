package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TarsArm extends SubsystemBase {

    // 2 motors of the base
    TalonFX baseMotorMain = new TalonFX(20);
    TalonFX baseMotorFollower = new TalonFX(22);

    // the wrist so that the position is always set static
    TalonFX wristMotor = new TalonFX(21);
    PositionDutyCycle stabilizingDutyCycle = new PositionDutyCycle(0);

    // PID constants
    double kP = .05;
    double kI = 0;
    double kD = 0;

    public TarsArm() {
        Slot0Configs motorConfigs = new Slot0Configs();
        motorConfigs.kP = kP;
        motorConfigs.kD = kD;
        motorConfigs.kI = kI;

        // apply pid constants
        baseMotorMain.getConfigurator().apply(motorConfigs);
        baseMotorFollower.getConfigurator().apply(motorConfigs);


        // set wrist and base to brake
        MotorOutputConfigs brakeConfigs = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
        baseMotorMain.getConfigurator().apply(brakeConfigs);
        baseMotorFollower.getConfigurator().apply(brakeConfigs);
        wristMotor.getConfigurator().apply(brakeConfigs);


        // reset encoder positions to 0
        baseMotorMain.setPosition(0);
        baseMotorFollower.setPosition(0);
        wristMotor.setPosition(0);

        // follower follows main motor
        baseMotorFollower.setControl(new Follower(baseMotorMain.getDeviceID(), false));


        // configure motion magic of base motors
        MotionMagicConfigs motionMagicConfigs = new TalonFXConfiguration().MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 1;
        motionMagicConfigs.MotionMagicAcceleration = 1;
        motionMagicConfigs.MotionMagicJerk = 1;

        baseMotorMain.getConfigurator().apply(motionMagicConfigs);
        baseMotorFollower.getConfigurator().apply(motionMagicConfigs);
    }

    @Override
    public void periodic() {
        System.out.println(baseMotorMain.getPosition().getValue());
    }

    // keeps the wrist position at 0
    void stabilizeWrist() {
        wristMotor.setControl(stabilizingDutyCycle);
    }

    // run base arm at percentage
    public void setBasePct(double pct) {
        baseMotorMain.set(pct);
        stabilizeWrist();
    }

    // sets base position using motion magic
    public void setBasePos(double pos) {

        MotionMagicVoltage positionReq = new MotionMagicVoltage(0);
        baseMotorMain.setControl(positionReq.withPosition(pos));
        stabilizeWrist();

    }
    
}
