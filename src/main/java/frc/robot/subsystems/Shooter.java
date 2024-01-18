package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    CANSparkFlex bottomMotor = new CANSparkFlex(Constants.bottomShooterMotorID, MotorType.kBrushless);
    CANSparkFlex topMotor = new CANSparkFlex(Constants.topShooterMotorID, MotorType.kBrushless);


    public Shooter() {

    }

    public void shoot(double power) {
        bottomMotor.set(-power);
        topMotor.set(power);
    }    
}
