package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private CANSparkFlex leftMotor, rightMotor;
    private SparkPIDController leftController, rightController;

    private TalonFX pivotMotor;
    private TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();    

    public Shooter() {
        leftMotor = new CANSparkFlex(Constants.BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkFlex(Constants.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        // rightMotor.setInverted(true);

        pivotMotor = new TalonFX(Constants.SHOOTER_PIVOT_MOTOR_ID);

        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotConfigs.Slot0.kV = 0;
        pivotConfigs.Slot0.kP = Constants.PIVOT_kP;
        pivotConfigs.Slot0.kI = Constants.PIVOT_kI;
        pivotConfigs.Slot0.kD = Constants.PIVOT_kD;

        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = 0.5;
        pivotConfigs.MotionMagic.MotionMagicAcceleration = 1;
        pivotConfigs.MotionMagic.MotionMagicJerk = 2;

        pivotMotor.getConfigurator().apply(pivotConfigs);


        // Set PID for left motor
        leftController = leftMotor.getPIDController();
        leftController.setP(.0005);
        leftController.setI(0.0);
        leftController.setD(0.0);
        leftController.setFF(0);
        leftController.setIZone(0);
        leftController.setOutputRange(-1, 1);

        // Set PID for right motor
        rightController = rightMotor.getPIDController();
        rightController.setP(0.0005);
        rightController.setI(0);
        rightController.setD(0);
        rightController.setFF(0);
        rightController.setIZone(0);
        rightController.setOutputRange(-1, 1);
    }


    public void shoot(double leftRpm, double rightRpm) {
        //set rpm for both motors
        leftController.setReference(leftRpm, ControlType.kVelocity);
        rightController.setReference(-rightRpm, ControlType.kVelocity);
        // rightController.setReference(rightRpm, ControlType.kVelocity); // TODO: check if this is correct based on setInverted


        // leftMotor.setSmartCurrentLimit(20, 5000);
        // rightMotor.setSmartCurrentLimit(20,3000);
    }   
    
    public void setShooterAngle(double angleOfShooter) {
        pivotMotor.setPosition(angleOfShooter);
    }

    public Command aimShooterAngleCommand(double angle) {
        return run(() -> setShooterAngle(angle));
    }

    public Command shootSpeakerCommand(double leftRPM, double rightRPM) {
        return run(() -> shoot(leftRPM, rightRPM))
        .finallyDo(() -> shoot(0, 0));
    }
}
