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
  public static final double J1_Encoder_Min = 1.4;

  public static final double J2_Encoder_Max = 5.3;
  public static final double J2_Encoder_Min = 3.0;
  
  public static final double J3_Encoder_Max = 6.15;
  public static final double J3_Encoder_Min = 2.2;
  
  // PID coefficients
  public static final double j1_maxV = 0.750;
  public static final double j1_maxAcc = 1.0;
  public static final double j1_kP = 8.0; 
  public static final double j1_kP_DOWN = 5.0; 
  public static final double j1_kI = 4.0;
  public static final double j1_kD = 0.4;
  public static final double j1_allE = 0.04;

  
  public static final double j2_maxV = 1.0;
  public static final double j2_maxAcc = 1.0;
  public static final double j2_kP = 15.0;
  public static final double j2_kP_DOWN = 3.0; 
  public static final double j2_kI = 8.5;
  public static final double j2_kD = 0.0;
  public static final double j2_allE = 0.04;

  
  public static final double j3_maxV = 0.75;
  public static final double j3_maxAcc = 1.0;
  public static final double j3_kP = 8.0;
  public static final double j3_kI = 6.5;
  public static final double j3_kD = 0.0;
  public static final double j3_allE = 0.04;
  
}