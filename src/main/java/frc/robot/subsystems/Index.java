package frc.robot.subsystems;

import static frc.robot.Constants.IndexConstants.INDEX_MOTOR_ID;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {
  CANSparkFlex indexMotor;
  DigitalInput beamBreak;

  public Index() {
    beamBreak = new DigitalInput(1);

    indexMotor = new CANSparkFlex(INDEX_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    indexMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Runs the index at a duty cycle percentage.
   *
   * @param pct the percentage to run the index at
   */
  public void runIndex(double pct) {
    indexMotor.set(pct);
  }

  /**
   * @return true if the beam is broken, false if not
   */
  public boolean beamBroken() {
    return beamBreak.get();
  }
}
