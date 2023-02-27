package frc.robot.subsystems.arm;

public final class ArmPositions {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ArmPositions() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  //Joint position arrays, {j1, j2, j3,} 


  public static final double[] TEST1 = {1.4,3.05,4.0 };
  public static final double[] TEST2 = {2.1,3.05,4.0};

  public static final double[] HOME = {2.1,5.3,5.4};
  public static final double[] READY = {0,0,0};
  public static final double[] APPROACH = {0,0,0};
  public static final double[] PICKUP = {1.4,3.050,4.0 };
}
