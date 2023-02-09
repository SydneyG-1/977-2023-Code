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

  public static final double J1_Encoder_Max = 0.350 * 360.0;
  public static final double J1_Encoder_Min = 0.26 * 360.0;

  public static final double J2_Encoder_Max = 330.0;
  public static final double J2_Encoder_Min = 300.0;

  // PID coefficients
  public static final double j1_kP = 0.03;
  public static final double j1_kI = 0.001;
  public static final double j1_kD = 0.0;
  public static final double j1_allE = 0.5;


  public static final double j2_kP = 0.0003;
  public static final double j2_kI = 0.00001;
  public static final double j2_kD = 0.0;
  public static final double j2_allE = 0.5;
}
