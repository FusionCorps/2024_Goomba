package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.INTAKE_MOTOR_ID;
import static frc.robot.Constants.diagnosticsTab;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkFlex intakeMotor;

  ShuffleboardTab tab = Shuffleboard.getTab("General");

  public GenericEntry isOuttaking =
      tab.add("Intake Out", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

  public GenericEntry isIntaking =
      tab.add("Intake In", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

  public Intake() {
    intakeMotor = new CANSparkFlex(INTAKE_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);

    isOuttaking.setBoolean(false);
    isIntaking.setBoolean(false);

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
