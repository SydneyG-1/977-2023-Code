package frc.robot.subsystems.gripper;

public final class GripperConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private GripperConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final int gripperSparkMaxID = 4;
  public static final double intakeSpeed = 0.5;
  public static final double intakeSpeedCone = 0.5;
  public static final double intakeSpeedCube = 0.5;
  public static final double releaseSpeed = 1.0;
  public static final int distanceValue = 1000;
}
