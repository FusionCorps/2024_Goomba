package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public class PIDWidget {
    double kP = 0.0, kI = 0.0, kD = 0.0, kFF = 0.0;
    PIDController pid = new PIDController(kP, kI, kD);
    public PIDWidget(ShuffleboardTab tab) {
        tab.add(pid).withWidget(BuiltInWidgets.kPIDController);
    }
}
