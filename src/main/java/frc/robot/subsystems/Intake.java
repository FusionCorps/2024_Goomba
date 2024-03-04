package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.INTAKE_MOTOR_ID;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intake extends SubsystemBase {
  @AutoLog
  public static class IntakeInputs {
    public double motorVoltage = 0.0;
    public double motorCurrent = 0.0;
  }

  private IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
  private CANSparkFlex intakeMotor;

  public Intake() {
    intakeMotor = new CANSparkFlex(INTAKE_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    inputs.motorVoltage = intakeMotor.getBusVoltage();
    inputs.motorCurrent = intakeMotor.getOutputCurrent();
  }

  /**
   * Runs the intake at a duty cycle percentage.
   *
   * @param pct the percentage to run the intake at
   */
  public void runIntake(double pct) {
    intakeMotor.set(pct);
  }

  @AutoLogOutput
  public double getIntakeOutput() {
    return intakeMotor.get();
  }
}
