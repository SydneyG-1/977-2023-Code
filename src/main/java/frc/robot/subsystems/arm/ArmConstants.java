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

  public static final double J1_Encoder_Max = 3.169;
  public static final double J1_Encoder_Min = 2.421;

  public static final double J2_Encoder_Max = 4.04;
  public static final double J2_Encoder_Min = 1.67;

  public static final double J3_Encoder_Max = 5.9;
  public static final double J3_Encoder_Min = 3.522;

  public static final double J2_J3_Interaction_Offset = 0.0;

  // PID coefficients
  public static final double j1_maxV = 0.35;
  public static final double j1_maxAcc = 1.00;
  public static final double j1_kP = 5.0;
  public static final double j1_kI = 4.0;
  public static final double j1_kD = 0.0;
  public static final double j1_allE = 0.04;

  public static final double j2_maxV = 1.00;
  public static final double j2_maxAcc = 1.0;
  public static final double j2_kP = 15.0;
  public static final double j2_kI = 0.1;
  public static final double j2_kD = 0.0;
  public static final double j2_allE = 0.04;

  public static final double j3_maxV = 1.0;
  public static final double j3_maxAcc = 1.0;
  public static final double j3_kP = 15.0;
  public static final double j3_kI = 0.0;
  public static final double j3_kD = 0.0;
  public static final double j3_allE = 0.04;
}
