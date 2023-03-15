/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.NavX.AHRS;

public class GyroIOPigeon2 implements GyroIO {
  // private final Pigeon2 gyro;
  private final AHRS gyro;
  private final double[] xyzDps = new double[3];

  public GyroIOPigeon2(int id) {
    // gyro = new AHRS(SerialPort.Port.kUSB);
    gyro = new AHRS(SPI.Port.kMXP, (byte) 50);
    // gyro = new Pigeon2(id, CAN_BUS_NAME);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {

    xyzDps[0] = gyro.getRawGyroX();
    xyzDps[1] = gyro.getRawGyroY();
    xyzDps[2] = gyro.getRawGyroZ();
    // zaccel = gyro.getRawAccelZ();

    inputs.connected = gyro.isConnected();
    // gyro.setAngleAdjustment(0);

    // inputs.positionDeg = -gyro.getAngle();
    inputs.positionDeg = -gyro.getYaw(); // degrees
    inputs.velocityDegPerSec = xyzDps[2];
    inputs.pitchDeg = gyro.getPitch();
    inputs.rollDeg = gyro.getRoll();
  }
}
