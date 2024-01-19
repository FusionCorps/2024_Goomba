package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    CANSparkFlex bottomMotor, topMotor;

    public Shooter() {
        bottomMotor = new CANSparkFlex(Constants.bottomShooterMotorID, MotorType.kBrushless);
        topMotor = new CANSparkFlex(Constants.topShooterMotorID, MotorType.kBrushless);
    }

    public void shoot(double power) {
        bottomMotor.set(-power);
        topMotor.set(power);
    }    
}
