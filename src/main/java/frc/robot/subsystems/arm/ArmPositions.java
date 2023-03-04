package frc.robot.subsystems.arm;

public final class ArmPositions {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ArmPositions() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  //Joint position arrays, {j1, j2, j3,} 

  //Joint 1 number decreases as arm goes FORWARD
  //Joint 2 number decreases as arm goes UP
  //Joint 3 number increases as arm goes UP

  public static final double[] TEST1 = {1.4,3.05,4.0 };
  public static final double[] TEST2 = {2.1,3.05,4.0};

  public static final double[] HOME = {2.108,5.32,5.296};
  public static final double[] HOME_APPROACH = {0,0,0};

  public static final double[] CONE_SHELF = {2.108,4.07,4.23};
  public static final double[] CONE_GROUND = {1.74,4.91,4.794};
  public static final double[] CONE_GROUND_INTERMEDIATE = {1.894,5.027,5.468};
  public static final double[] CONE_HOME = {2.108,5.32,5.53};
  public static final double[] CONE_LOW = {1.753,4.726,5.664};  
  public static final double[] CONE_MID = {1.879,3.807,4.346};
  public static final double[] CONE_MID_INTERMEDIATE = {2.108,4.070,4.266};
  public static final double[] CONE_HIGH = {1.802,3.788,4.217};

  public static final double[] CUBE_SHELF = {2.108,4.076,4.242};
  public static final double[] CUBE_GROUND =  {1.587,4.875,5.371};
  public static final double[] CUBE_HOME = {2.108,5.32,5.53};
  public static final double[] CUBE_LOW = {2.108,5.32,5.62};
  public static final double[] CUBE_MID_INTERMEDIATE = {2.108,4.536,4.88};
  public static final double[] CUBE_HIGH = {1.838,3.788,4.217};
  public static final double[] CUBE_MID = {2.108,4.389,4.591};
  
}
