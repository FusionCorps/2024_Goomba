package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {

  CANSparkFlex indexMotor;

  public static DigitalInput beam_break;

  public Index() {

    indexMotor = new CANSparkFlex(4, CANSparkFlex.MotorType.kBrushless);
    indexMotor.setIdleMode(IdleMode.kBrake);

    
  }

  

  public void indexIn(double pct) {

    indexMotor.set(pct);
  }
}
