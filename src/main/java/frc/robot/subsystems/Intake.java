package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkFlex intakeMotor;

  public Intake() {
    intakeMotor = new CANSparkFlex(Constants.INTAKE_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
  }

  public void runIntake(double pct) {
    intakeMotor.set(pct);
  }
}
