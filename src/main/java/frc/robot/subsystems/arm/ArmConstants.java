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

  public static final double J2_Encoder_Max = 5.486;
  public static final double J2_Encoder_Min = 2.74;

  public static final double J3_Encoder_Max = 5.848;
  public static final double J3_Encoder_Min = 3.322;

  // PID coefficients
  public static final double j1_maxV = 0.750;
  public static final double j1_maxAcc = 1.0;
  public static final double j1_kP = 8.0;
  public static final double j1_kI = 4.0;
  public static final double j1_kD = 0.4;
  public static final double j1_allE = 0.04;

  public static final double j2_maxV = 1.0;
  public static final double j2_maxAcc = 2.0;
  public static final double j2_kP = 20.0;
  public static final double j2_kI = 0.5;
  public static final double j2_kD = 0.01;
  public static final double j2_allE = 0.04;

  public static final double j3_maxV = 1.0;
  public static final double j3_maxAcc = 2.0;
  public static final double j3_kP = 10.0;
  public static final double j3_kI = 0.5;
  public static final double j3_kD = 0.01;
  public static final double j3_allE = 0.04;
}
