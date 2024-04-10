package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class UtilFunctions {
  public static Alliance getAllianceColor() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }
}
