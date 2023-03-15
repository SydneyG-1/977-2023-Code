package frc.robot.subsystems.arm;

public final class ArmPositions {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ArmPositions() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  // Joint position arrays, {j1, j2, j3,}

  // Joint 1 number decreases as arm goes FORWARD
  // Joint 2 number increase as arm goes UP
  // Joint 3 number increases as arm goes UP

  public static final double[] TEST1 = {1.4, 3.05, 4.0};
  public static final double[] TEST2 = {2.1, 3.05, 4.0};

  public static final double[] HOME = {3.169, 2.75, 5.22};
  public static final double[] HOME_APPROACH = {0, 0, 0};

  public static final double[] CONE_SHELF = {3.17, 3.98, 4.3};
  public static final double[] CONE_GROUND = {3.169, 2.75, 5.22};
  public static final double[] CONE_GROUND_INTERMEDIATE = {3.169, 2.75, 5.22};
  public static final double[] CONE_HOME = {3.169, 2.75, 5.22};
  public static final double[] CONE_LOW = {3.169, 2.75, 5.22};
  public static final double[] CONE_MID = {3.169, 2.75, 5.22};
  public static final double[] CONE_MID_INTERMEDIATE = {3.17, 3.98, 4.3};
  public static final double[] CONE_HIGH = {2.75, 3.98, 4.3};

  public static final double[] CUBE_SHELF = {3.17, 3.98, 4.3};
  public static final double[] CUBE_GROUND = {3.169, 2.75, 5.22};
  public static final double[] CUBE_HOME = {3.169, 2.75, 5.22};
  public static final double[] CUBE_LOW = {3.169, 2.75, 5.22};
  public static final double[] CUBE_MID_INTERMEDIATE = {3.169, 2.75, 5.22};
  public static final double[] CUBE_HIGH = {3.169, 2.75, 5.22};
  public static final double[] CUBE_MID = {3.169, 2.75, 5.22};
}
