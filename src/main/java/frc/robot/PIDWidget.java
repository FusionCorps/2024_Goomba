package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDWidget {
  String kPName, kIName, kDName, kFFName;

  public PIDWidget(double kP, double kI, double kD, String name) {
    kPName = name + " kP";
    kIName = name + " kI";
    kDName = name + " kD";
    SmartDashboard.putNumber(kPName, kP);
    SmartDashboard.putNumber(kIName, kI);
    SmartDashboard.putNumber(kDName, kD);
  }

  public void updatePIDF(PIDController pid) {
    StringBuilder out = new StringBuilder();
    if (getkP() != pid.getP()) {
      out.append("NEW: ");
      pid.setP(getkP());
      out.append("kP = " + pid.getP() + " ");
    }
    if (getkI() != pid.getI()) {
      pid.setI(getkI());
      out.append("kI = " + pid.getI() + " ");
    }
    if (getkD() != pid.getD()) {
      pid.setD(getkD());
      out.append("kD = " + pid.getD());
    }
    if (!out.isEmpty()) System.out.println(out.toString());
  }

  public double getkP() {
    return SmartDashboard.getNumber(kPName, 0.0);
  }

  public double getkI() {
    return SmartDashboard.getNumber(kIName, 0.0);
  }

  public double getkD() {
    return SmartDashboard.getNumber(kDName, 0.0);
  }
}
