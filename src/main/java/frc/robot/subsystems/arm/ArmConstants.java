package frc.robot.subsystems.arm;

public final class ArmConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ArmConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final int J1_motorSparkMaxID = 1; // Setup ID's Later
  public static final int J2_motorSparkMaxID = 2; // Setup ID's Later
  public static final int J3_motorSparkMaxID = 3; // Setup ID's Later
  public static final int J4_motorSparkMaxID = 4; // Setup ID's Later

  public static final double J1_Encoder_Max = 2.05;
  public static final double J1_Encoder_Min = 1.0;

  public static final double J2_Encoder_Max = 5.3;
  public static final double J2_Encoder_Min = 3.0;
  
  public static final double J3_Encoder_Max = 5.6;
  public static final double J3_Encoder_Min = 4.0;
  
  public static final double J4_Encoder_Max = 330.0;
  public static final double J4_Encoder_Min = 2200.0;

  // PID coefficients
  public static final double j1_maxV = 0.50;//rad per sec
  public static final double j1_maxAcc = 1.0;//rad per sec per sec
  public static final double j1_kP = 8.0; 
  public static final double j1_kP_DOWN = 5.50; 
  public static final double j1_kI = 1.0;
  public static final double j1_kD = 0.4;
  public static final double j1_allE = 0.02;
  public static final double j1_ks = 1.290;//volts
  public static final double j1_kg = 1.223; //volts
  public static final double j1_kv = 0.281;//volts*s/rad
  public static final double j1_ka = 0.361;//volts*s^2/rad

  
  public static final double j2_maxV = .75;
  public static final double j2_maxAcc = 1.0;
  public static final double j2_kP = 3.0;
  public static final double j2_kP_DOWN = 0.2; 
  public static final double j2_kI = 0.5;
  public static final double j2_kD = 0.4;
  public static final double j2_allE = 0.02;
  public static final double j2_ks = 0.0;
  public static final double j2_kg = 0.0;
  public static final double j2_kv = 0.0;
  public static final double j2_ka = 0.0;

  
  public static final double j3_maxV = 7.5;
  public static final double j3_maxAcc = 5.0;
  public static final double j3_kP = 6.0;
  public static final double j3_kI = 0.5;
  public static final double j3_kD = 0.0;
  public static final double j3_allE = 0.01;
  public static final double j3_ks = 0.0;
  public static final double j3_kg = 0.0;
  public static final double j3_kv = 0.0;
  public static final double j3_ka = 0.0;
  
  /* 
  public static final double j4_maxV = 10.0;
  public static final double j4_maxAcc = 1.0;
  public static final double j4_kP = 0.003;
  public static final double j4_kI = 0.00001;
  public static final double j4_kD = 0.0;
  public static final double j4_allE = 0.5;
  public static final double j4_ks = 0.0;
  public static final double j4_kg = 0.00;
  public static final double j4_kv = 0.00;
  public static final double j4_ka = 0.00;
  */
}
