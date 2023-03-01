package frc.robot.subsystems.arm;

public final class ArmPositions {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ArmPositions() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  //Joint position arrays, {j1, j2, j3,} 


  public static final double[] TEST1 = {1.4,3.05,4.0 };
  public static final double[] TEST2 = {2.1,3.05,4.0};

  public static final double[] HOME = {2.108,5.32,5.296};
  public static final double[] HOME_APPROACH = {0,0,0};

  public static final double[] CONE_SHELF = {2.108,4.07,4.23};
  public static final double[] CONE_GROUND = {1.74,4.91,4.794};
  public static final double[] CONE_HOME = {2.108,5.32,5.53};
  public static final double[] CONE_LOW = {1.955,4.696,4.922};
  public static final double[] CONE_MID = {1.802,3.788,4.217};
  public static final double[] CONE_MID_INTERMEDIATE = {2.108,4.070,4.266};
  public static final double[] CONE_HIGH = {1.802,3.788,4.217};

  public static final double[] CUBE_SHELF = {2.108,4.076,4.242};
  public static final double[] CUBE_GROUND =  {2.108,5.063,4.315};
  public static final double[] CUBE_HOME = {2.108,5.32,5.53};
  public static final double[] CUBE_LOW = {2.108,5.32,5.62};
  public static final double[] CUBE_MID_INTERMEDIATE = {2.108,4.536,4.88};
  public static final double[] CUBE_HIGH = {1.838,3.788,4.217};
  public static final double[] CUBE_MID = {2.108,4.389,4.591};
  
}
