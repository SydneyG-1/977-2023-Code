package frc.robot.subsystems.arm;

public final class ArmConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ArmConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final int J1_motorSparkMaxID = 1;
  public static final int J2_motorSparkMaxID = 2;
  public static final int J3_motorSparkMaxID = 3;
  public static final int J4_motorSparkMaxID = 4;

  public static final double J1_Encoder_Max = 3.1;
  public static final double J1_Encoder_Min = 2.55;

  public static final double J2_Encoder_Max = 4.04;
  public static final double J2_Encoder_Min = 1.6;

  public static final double J3_Encoder_Max = 5.9;
  public static final double J3_Encoder_Min = 3.522;

  // offset for J1 to read 0 rad when at home
  public static final double J1_offset = 3.1;

  // offset for J2 to read 0 rad when at home
  public static final double J2_offset = 1.67;

  // offset for J3 to be level and read 0 radians at home
  public static final double J3_offset = 5.487;

  // PID coefficients
  public static final double j1_maxV = 0.250;
  public static final double j1_maxAcc = 1.0;
  public static final double j1_kP = 40.0;
  public static final double j1_kI = 0.0;
  public static final double j1_kD = 0.0;
  public static final double j1_allE = 0.09;

  public static final double j2_maxV = 1.0;
  public static final double j2_maxAcc = 1.0;
  public static final double j2_kP = 18.0;
  public static final double j2_kI = 0.0;
  public static final double j2_kD = 0.0;
  public static final double j2_allE = 0.09;

  public static final double j3_maxV = 1.0;
  public static final double j3_maxAcc = 2.0;
  public static final double j3_kP = 10.0;
  public static final double j3_kI = 10.0;
  public static final double j3_kD = 1.0;
  public static final double j3_allE = 0.09;

  public static final double j1_ks = 0.0;
  public static final double j1_kv = 0.0;
  public static final double j1_kg = 0.0;
  public static final double j1_ka = 0.0;

  public static final double j2_ks = 0.17;
  public static final double j2_kv = 3.98;
  public static final double j2_kg = 0.67;
  public static final double j2_ka = 0.05;

  public static final double j3_ks = 0.0;
  public static final double j3_kv = 0.0;
  public static final double j3_kg = 0.0;
  public static final double j3_ka = 0.0;
}
