/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class GyroIOPigeon2 implements GyroIO {
  // private final Pigeon2 gyro;
  private final AHRS gyro;
  private final double[] xyzDps = new double[3];

  public GyroIOPigeon2(int id) {

    gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
    // gyro = new Pigeon2(id, CAN_BUS_NAME);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {

    xyzDps[0] = gyro.getRawGyroX();
    xyzDps[1] = gyro.getRawGyroY();
    xyzDps[2] = gyro.getRawGyroZ();
    inputs.connected = gyro.isConnected();
    inputs.positionDeg = -gyro.getYaw(); // degrees
    inputs.velocityDegPerSec = xyzDps[2]; // degrees per second
  }
}
