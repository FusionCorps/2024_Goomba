package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    CANSparkFlex leftMotor, rightMotor;
    SparkPIDController leftController, rightController;

    public Shooter() {
        leftMotor = new CANSparkFlex(Constants.BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkFlex(Constants.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);

        leftController = leftMotor.getPIDController();
        leftController.setP(.0005);
        leftController.setI(0.0);
        leftController.setD(0.0);
        leftController.setFF(0);
        leftController.setIZone(0);
        leftController.setOutputRange(-1, 1);

        rightController = rightMotor.getPIDController();
        rightController.setP(0.0005);
        rightController.setI(0);
        rightController.setD(0);
        rightController.setFF(0);
        rightController.setIZone(0);
        rightController.setOutputRange(-1, 1);
    }

    public void shoot(double leftRpm, double rightRpm) {
        
        
        leftController.setReference(leftRpm, ControlType.kVelocity);
        rightController.setReference(rightRpm, ControlType.kVelocity);

        // leftMotor.setSmartCurrentLimit(20, 5000);
        // rightMotor.setSmartCurrentLimit(20,3000);
        // rightMotor.set(-rightRpm);
        // leftMotor.set(leftRpm);
    }    
}
