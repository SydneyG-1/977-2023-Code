package frc.robot.subsystems.arm;

public final class ArmPositions {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ArmPositions() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  //Joint position arrays, {j1, j2, j3,} 


  public static final double[] TEST1 = {113,240, 290};
  public static final double[] TEST2 = {80,190,320};

  public static final double[] HOME = {120,100,0};
  public static final double[] READY = {0,0,0};
  public static final double[] APPROACH = {0,0,0};
  public static final double[] PICKUP = {0,0,0};
}
