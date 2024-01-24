package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    CANSparkFlex leftMotor, rightMotor;
    SparkPIDController leftController, rightController;

    TalonFX pivotMotor;
    TalonFXConfiguration pivotPID = new TalonFXConfiguration();
    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    

    public Shooter() {
        leftMotor = new CANSparkFlex(Constants.BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkFlex(Constants.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);

        pivotMotor = new TalonFX(Constants.SHOOTER_PIVOT_MOTOR_ID);

        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        pivotPID.Slot0.kV = 0;
        pivotPID.Slot0.kP = Constants.PIVOT_kP;
        pivotPID.Slot0.kI = Constants.PIVOT_kI;
        pivotPID.Slot0.kD = Constants.PIVOT_kD;

        pivotMotor.getConfigurator().apply(pivotPID);

        motionMagic.MotionMagicCruiseVelocity = 0.5;
        motionMagic.MotionMagicAcceleration = 1;
        motionMagic.MotionMagicJerk = 2;
        pivotMotor.getConfigurator().apply(motionMagic);


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

        // leftMotor.setSmartCurrentLimit(20, 5000);
        // rightMotor.setSmartCurrentLimit(20,3000);
        // rightMotor.set(-rightRpm);
        // leftMotor.set(leftRpm);
    }   
    
    public void setShooterAngle(double angleOfShooter){

    

        pivotMotor.set(angleOfShooter);

    }
}
