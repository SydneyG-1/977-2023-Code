// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  // private DutyCycleEncoder J1_Encoder = new DutyCycleEncoder(0);
  // private DutyCycleEncoder J2_Encoder = new DutyCycleEncoder(1);
  private AbsoluteEncoder J1_Encoder = J1_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private AbsoluteEncoder J2_Encoder = J2_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private AbsoluteEncoder J3_Encoder = J3_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private AbsoluteEncoder J4_Encoder = J4_motor.getAbsoluteEncoder(Type.kDutyCycle);

  private ProfiledPIDController j1Controller;
  private TrapezoidProfile.Constraints j1Profile = new TrapezoidProfile.Constraints(5.0, 1.0);
  private double j1goal = 0.0;
  private double temp = 0.0;
  /** Creates a new Arm. */
  public Arm() {
    super();
     J1_motor.setInverted(true);
    J1_motor.enableSoftLimit(SoftLimitDirection.kForward, false);
    J1_motor.enableSoftLimit(SoftLimitDirection.kReverse, false);

    J1_Encoder.setPositionConversionFactor(360.0);
    J2_Encoder.setPositionConversionFactor(360.0);
    // J1_motor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.J1_Encoder_Max);
    // J1_motor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.J1_Encoder_Min);

    j1Controller =
        new ProfiledPIDController(
            ArmConstants.j1_kP, ArmConstants.j1_kI, ArmConstants.j1_kD, j1Profile, 0.02);
    j1Controller.setTolerance(ArmConstants.j1_allE);
  }

  public double getJ1postition() {
    // return J1_Encoder.getAbsolutePosition();
    return J1_Encoder.getPosition();
  }

  public double getJ2postition() {
    // return J2_Encoder.getAbsolutePosition();
    return J2_Encoder.getPosition();
  }

  public void moveJ1Up() {
    if (J1_Encoder.getPosition() < ArmConstants.J1_Encoder_Max) {
      J1_motor.set(.15);
    } else {
      J1_motor.set(0.0);
    }
  }

  public void moveJ1Dwn() {
    if (J1_Encoder.getPosition() > ArmConstants.J1_Encoder_Min) {
      J1_motor.set(-.15);
    } else {
      J1_motor.set(0.0);
    }
  }

  public void stopJ1() {
    J1_motor.set(0.0);
  }

  public void resetJ1Position() {
    j1Controller.reset(getJ1postition());
  }

  public void moveJ1(double positionGoal) {

    if (positionGoal < ArmConstants.J1_Encoder_Max && positionGoal > ArmConstants.J1_Encoder_Min) {
      j1goal = positionGoal;
      j1Controller.setGoal(positionGoal);
    }
    temp = j1Controller.calculate(getJ1postition());
    J1_motor.set(temp);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("J1 Output", J1_motor.getAppliedOutput());

    SmartDashboard.putNumber("j1positionSub", getJ1postition());
    SmartDashboard.putNumber("j1goal", j1goal);
    SmartDashboard.putNumber("j1pidCalc", temp);
    SmartDashboard.putNumber("j1piderror", j1Controller.getPositionError());

    SmartDashboard.putNumber("j1pidsetP", j1Controller.getSetpoint().position);
    // This method will be called once per scheduler run
  }
}
