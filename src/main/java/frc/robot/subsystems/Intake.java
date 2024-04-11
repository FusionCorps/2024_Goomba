package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.INTAKE_MOTOR_ID;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFX intakeMotor;

  ShuffleboardTab tab = Shuffleboard.getTab("General");

  public GenericEntry isOuttaking =
      tab.add("Intake Out", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

  public GenericEntry isIntaking =
      tab.add("Intake In", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

  public Intake() {
    intakeMotor = new TalonFX(INTAKE_MOTOR_ID);
    intakeMotor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentThreshold(80)
                .withSupplyTimeThreshold(0.5));

    isOuttaking.setBoolean(false);
    isIntaking.setBoolean(false);
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
