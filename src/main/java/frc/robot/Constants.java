package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Constants {
  public static final int TOP_SHOOTER_MOTOR_ID = 3; // TODO: change this id
  public static final int BOTTOM_SHOOTER_MOTOR_ID = 2; // TODO: change this id
  public static final int SHOOTER_PIVOT_MOTOR_ID = -1; // TODO: change this id

  public static final double PIVOT_kP = 0.2;
  public static final double PIVOT_kD = 0;
  public static final double PIVOT_kI = 0;

  public static final double PIVOT_START_POS = Units.degreesToRotations(30);
  public static final double PIVOT_CLIMB_UP_POS = Units.degreesToRotations(120);
  public static final double PIVOT_CLIMB_DOWN_POS = Units.degreesToRotations(40);

  public static final String LIMELIGHT_NAME = "limelight";
  public static final double LIMELIGHT_TX_RANGE_DEG = 29.8; // range is thus -29.8 to 29.8
  public static final double LIMELIGHT_TY_RANGE_DEG = 24.85; // range is thus -24.85 to 24.85

  public static final double camHeightMeters = Units.inchesToMeters(54); // dont change
  public static final double cameraPitchRadians = Units.degreesToRadians(1.0); // dont change
  // 48 1/8 to bottom (white boundary edge) of target, 5 1/4 to center of target
  public static final double sourceTargetHeightMeters = Units.inchesToMeters(53.375);
  // 51 7/8 to bottom of target, 5 1/4 to center of target
  public static final double speakerTargetHeightMeters = Units.inchesToMeters(57.125);
  // 48 1/8 to bottom (white boundary edge) of target, 5 1/4 to center of target
  public static final double ampTargetHeightMeters =
      Units.inchesToMeters(53.375); // identical height to source
  // 47.5 inches to bottom of target, 4.5 to center of target
  public static final double trapTargetHeightMeters = Units.inchesToMeters(52);

  public static DriverStation.Alliance allianceColor = null; // null by default, set in Robot.java
  public static int allianceLocation = -1; // -1 by default, set in Robot.java\

  public static ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");
}
