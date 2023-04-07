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

  //NEW positions with home at 0 radians 
  public static final double[] N_HOME =       {0.0,   0.0,  0.0};

  public static final double[] N_CONE_SHELF = {0.0,   1.22, 0.0};
  public static final double[] N_CUBE_SHELF = {0.0,   1.25, -0.5};

  public static final double[] N_CONE_GROUND_INTERMEDIATE = {0.0,   0.37,   0.0};
  public static final double[] N_CUBE_GROUND_INTERMEDIATE = {0.0,   0.37,   -0.15};

  public static final double[] N_CONE_GROUND = {0.2,   0.247,   0.0 };
  public static final double[] N_CUBE_GROUND = {0.1,   0.218,   -0.35 };

  public static final double[] N_CONE_HIGH_INTERMEDIATE =  {0.0,  1.38, 0.0};
  public static final double[] N_CUBE_HIGH_INTERMEDIATE =  {0.0,  1.18, 0.1};

  public static final double[] N_CONE_HIGH =  {0.5,  1.6, 0.55};
  public static final double[] N_CUBE_HIGH =  {0.40,  1.28, 0.1};

  public static final double[] N_CONE_HOME =  {0.0,  0.0, 0.15};
  public static final double[] N_CUBE_HOME =  {0.0,  0.0, 0.0};

  public static final double[] N_CONE_MID =  {0.0,  1.1, 0.35};
  public static final double[] N_CUBE_MID =  {0.0,  0.8, 0.1};

  public static final double[] N_CONE_LOW =  {0.0,  0.3, 0.0};
  public static final double[] N_CUBE_LOW =  {0.0,  0.25, 0.0};




  public static final double[] TEST1 = {1.4, 3.05, 4.0};
  public static final double[] TEST2 = {2.1, 3.05, 4.0};

  public static final double[] HOME = {3.181, 1.67, 5.487};


  public static final double[] CONE_SHELF_NEW = {3.168, 2.85, 5.487};
  public static final double[] CONE_HIGH_NEW = {2.77, 2.85, 5.487};

  public static final double[] CONE_SHELF = {3.168, 2.85, 4.3507};
  public static final double[] CONE_GROUND = {2.531, 2.117, 5.725};
  public static final double[] CONE_GROUND_INTERMEDIATE = {3.168, 2.04, 4.99};
  public static final double[] CONE_HOME = {3.168, 1.67, 5.687};//{3.162, 1.674, 5.487};
  public static final double[] CONE_LOW = {3.168, 2.047, 4.75};
  public static final double[] CONE_LOW_INTERMEDIATE = {3.168, 2.047, 4.75};
  public static final double[] CONE_MID = {3.168, 2.8, 4.517};//{3.162, 2.9, 4.517};
  public static final double[] CONE_MID_SCORE = {2.924, 2.8, 4.5517};
  public static final double[] CONE_MID_INTERMEDIATE_WAY_UP ={3.172, 2.8, 4.75};
  public static final double[] CONE_MID_INTERMEDIATE_WAY_UP2 ={2.9, 3.034, 4.5};
  public static final double[] CONE_MID_INTERMEDIATE_WAY_UP3 ={2.7, 3.5, 4.313};
  public static final double[] CONE_MID_INTERMEDIATE = {3.162, 3.034, 4.313};
  public static final double[] CONE_MID_INTERMEDIATE_WAY_DOWN = {3.168, 3.434, 4.313};
  public static final double[] CONE_HIGH ={2.77, 3.642, 4.101};// {2.421, 4.042, 4.101};
  public static final double[] CONE_HIGH_SCORE = {2.421, 3.821, 4.285};

  public static final double[] CUBE_SHELF = {3.168, 2.92, 4.05};
  public static final double[] CUBE_GROUND = {2.697, 1.988, 5.253};
  public static final double[] CUBE_GROUND_INTERMEDIATE = {3.168, 2.04, 4.99};
  public static final double[] CUBE_HOME = {3.168, 1.67, 5.487};
  public static final double[] CUBE_LOW = {3.168, 1.673, 5.425};
  public static final double[] CUBE_MID_INTERMEDIATE = {3.168, 1.685, 5.425};
  public static final double[] CUBE_HIGH =  {2.77, 3.4, 4.301};//{2.777, 3.369, 4.371};
  public static final double[] CUBE_HIGH_SHOT = {3.168, 3.23, 4.5};
  public static final double[] CUBE_MID = {3.168, 2.6, 4.468}; 
  public static final double[] CUBE_HIGH_INTERMEDIATE = {3.168, 2.8, 4.75};//{3.162,  2.844, 4.254};
  public static final double[] CUBE_HIGH_INTERMEDIATE_WAY_DOWN = {3.168,  2.844,  4.254};


  
}
