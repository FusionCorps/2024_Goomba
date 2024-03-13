package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.INTAKE_MOTOR_ID;
import static frc.robot.Constants.diagnosticsTab;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkFlex intakeMotor;

  

  public Intake() {
    intakeMotor = new CANSparkFlex(INTAKE_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    

  
    
  
    

    diagnosticsTab.addDouble("Intake Output %", () -> intakeMotor.getAppliedOutput());
  }

  /**
   * Runs the intake at a duty cycle percentage.
   *
   * @param pct the percentage to run the intake at
   */
  public void runIntake(double pct) {
    intakeMotor.set(pct);
  }
}
