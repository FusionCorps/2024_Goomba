package frc.robot.subsystems;

import static frc.robot.Constants.IndexConstants.INDEX_MOTOR_ID;
import static frc.robot.Constants.diagnosticsTab;
import static frc.robot.Constants.driverTab;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {
  CANSparkFlex indexMotor;
  public static DigitalInput beamBreak;

  ShuffleboardTab tab = Shuffleboard.getTab("General");

  public GenericEntry isTrapping = tab.add("Is Trapping", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

  public Index() {
    beamBreak = new DigitalInput(1);

    indexMotor = new CANSparkFlex(INDEX_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    indexMotor.setIdleMode(IdleMode.kBrake);

    isTrapping.setBoolean(false);

    driverTab.addBoolean("Note Ready", this::beamBroken).withSize(2, 2).withPosition(6, 0);

    diagnosticsTab.addDouble("Index Output %", () -> indexMotor.getAppliedOutput());
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
  public boolean beamBroken() {
    return !beamBreak.get(); // sensor actually returns true when beam is detected, so inverting it
  }
}
