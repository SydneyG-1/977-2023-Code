// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private CANSparkMax J1_motor =
      new CANSparkMax(ArmConstants.J1_motorSparkMaxID, MotorType.kBrushless);
  private CANSparkMax J2_motor =
      new CANSparkMax(ArmConstants.J2_motorSparkMaxID, MotorType.kBrushless);
  private CANSparkMax J3_motor =
      new CANSparkMax(ArmConstants.J3_motorSparkMaxID, MotorType.kBrushless);
  private CANSparkMax J4_motor =
      new CANSparkMax(ArmConstants.J4_motorSparkMaxID, MotorType.kBrushless);

  //private DutyCycleEncoder J1_Encoder = new DutyCycleEncoder(0);
  //private DutyCycleEncoder J2_Encoder = new DutyCycleEncoder(1);
  private AbsoluteEncoder J1_Encoder = J1_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private AbsoluteEncoder J2_Encoder = J2_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private AbsoluteEncoder J3_Encoder = J3_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private AbsoluteEncoder J4_Encoder = J4_motor.getAbsoluteEncoder(Type.kDutyCycle);

  /** Creates a new Arm. */
  public Arm() {}

  public double getJ1postition() {
    //return J1_Encoder.getAbsolutePosition();
    return J1_Encoder.getPosition();
  }

  public double getJ2postition() {
    //return J2_Encoder.getAbsolutePosition();
    return J2_Encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
