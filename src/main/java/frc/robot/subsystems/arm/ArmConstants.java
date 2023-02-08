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

  public static final double J1_Encoder_Max = 0.350;
  public static final double J1_Encoder_Min = 0.26;

  public static final double J2_Encoder_Max = 0.015;
  public static final double J2_Encoder_Min = 0.561;
}
