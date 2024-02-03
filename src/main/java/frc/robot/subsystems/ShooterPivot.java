package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterPivot extends SubsystemBase {

    TalonFX pivotMotor;

    public ShooterPivot() {

        pivotMotor = new TalonFX(Constants.PIVOT_ID);

        Slot0Configs motorConfigs = new Slot0Configs();
        motorConfigs.kP = Constants.PIDConstants.pivotKP;
        motorConfigs.kD = Constants.PIDConstants.pivotKD;
        motorConfigs.kI = Constants.PIDConstants.pivotKI;

        // apply pid constants
        pivotMotor.getConfigurator().apply(motorConfigs);


        // set wrist and base to brake
        MotorOutputConfigs brakeConfigs = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
        pivotMotor.getConfigurator().apply(brakeConfigs);


        // reset encoder positions to 0
        pivotMotor.setPosition(0);
     
        // follower follows main motor
        //baseMotorFollower.setControl(new Follower(baseMotorMain.getDeviceID(), false));


        // configure motion magic of base motors
        MotionMagicConfigs motionMagicConfigs = new TalonFXConfiguration().MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 50;
        motionMagicConfigs.MotionMagicAcceleration = 20;
        motionMagicConfigs.MotionMagicJerk = 10;

        pivotMotor.getConfigurator().apply(motionMagicConfigs);

    }
    
}
