package frc.robot.subsystems;

import static frc.robot.Constants.IndexConstants.INDEX_MOTOR_ID;
import static frc.robot.Constants.driverTab;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public class Index extends SubsystemBase {
  @AutoLog
  public static class IndexInputs {
    public double motorVoltage = 0.0;
    public double motorCurrent = 0.0;
  }

  public IndexInputsAutoLogged inputs = new IndexInputsAutoLogged();

  CANSparkFlex indexMotor;
  public static DigitalInput beamBreak;

  public Index() {
    beamBreak = new DigitalInput(1);

    indexMotor = new CANSparkFlex(INDEX_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    indexMotor.setIdleMode(IdleMode.kBrake);

    driverTab.addBoolean("Note Ready", this::beamBroken).withSize(2, 2).withPosition(6, 0);
  }

  @Override
  public void periodic() {
    inputs.motorVoltage = indexMotor.getBusVoltage();
    inputs.motorCurrent = indexMotor.getOutputCurrent();
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
   * @return true when note is loaded, false when no note detected
   */
  @AutoLogOutput
  public boolean beamBroken() {
    return !beamBreak.get(); // sensor actually returns true when beam is detected, so inverting it
  }

  @AutoLogOutput
  public double getIndexOutput() {
    return indexMotor.get();
  }
}
