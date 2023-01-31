// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax joint1 = new CANSparkMax(1, MotorType.kBrushless);

  private RelativeEncoder joint1Encoder = joint1.getAlternateEncoder(8192);

  public Arm() {}

  public double getAngle() {
    return joint1Encoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Joint1 Position", getAngle());
    // This method will be called once per scheduler run
  }
}
