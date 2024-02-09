package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {

  CANSparkFlex indexMotor;

  public Index() {

    indexMotor = new CANSparkFlex(4, CANSparkFlex.MotorType.kBrushless);
  }

  public void indexIn(double pct) {
    indexMotor.set(pct);
  }
}
