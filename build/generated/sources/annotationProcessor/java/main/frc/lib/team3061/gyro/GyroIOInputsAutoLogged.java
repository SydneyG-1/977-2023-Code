package frc.lib.team3061.gyro;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("PositionDeg", positionDeg);
    table.put("VelocityDegPerSec", velocityDegPerSec);
    table.put("PitchDeg", pitchDeg);
    table.put("RollDeg", rollDeg);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.getBoolean("Connected", connected);
    positionDeg = table.getDouble("PositionDeg", positionDeg);
    velocityDegPerSec = table.getDouble("VelocityDegPerSec", velocityDegPerSec);
    pitchDeg = table.getDouble("PitchDeg", pitchDeg);
    rollDeg = table.getDouble("RollDeg", rollDeg);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.positionDeg = this.positionDeg;
    copy.velocityDegPerSec = this.velocityDegPerSec;
    copy.pitchDeg = this.pitchDeg;
    copy.rollDeg = this.rollDeg;
    return copy;
  }
}
