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

  private AbsoluteEncoder J1_Encoder = J1_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private AbsoluteEncoder J2_Encoder = J2_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private AbsoluteEncoder J3_Encoder = J3_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private AbsoluteEncoder J4_Encoder = J4_motor.getAbsoluteEncoder(Type.kDutyCycle);

  private ProfiledPIDController j1Controller;
  private TrapezoidProfile.Constraints j1Profile = new TrapezoidProfile.Constraints(ArmConstants.j1_maxV, ArmConstants.j1_maxAcc);
 

  private ProfiledPIDController j2Controller;
  private TrapezoidProfile.Constraints j2Profile = new TrapezoidProfile.Constraints(ArmConstants.j2_maxV, ArmConstants.j2_maxAcc);


  private ProfiledPIDController j3Controller;
  private TrapezoidProfile.Constraints j3Profile = new TrapezoidProfile.Constraints(ArmConstants.j3_maxV, ArmConstants.j3_maxAcc);


  private ProfiledPIDController j4Controller;
  private TrapezoidProfile.Constraints j4Profile = new TrapezoidProfile.Constraints(ArmConstants.j4_maxV, ArmConstants.j4_maxAcc);

  /** Creates a new Arm. */
  public Arm() {
    super();

    J1_motor.setInverted(true);
    J1_motor.enableSoftLimit(SoftLimitDirection.kForward, false);
    J1_motor.enableSoftLimit(SoftLimitDirection.kReverse, false);

    J1_Encoder.setPositionConversionFactor(360.0);
    J2_Encoder.setPositionConversionFactor(360.0);
    J3_Encoder.setPositionConversionFactor(360.0);
    J4_Encoder.setPositionConversionFactor(360.0);


    j1Controller =
        new ProfiledPIDController(
            ArmConstants.j1_kP, ArmConstants.j1_kI, ArmConstants.j1_kD, j1Profile, 0.02);
    j1Controller.setTolerance(ArmConstants.j1_allE);

    j2Controller =
        new ProfiledPIDController(
            ArmConstants.j2_kP, ArmConstants.j2_kI, ArmConstants.j2_kD, j2Profile, 0.02);
    j2Controller.setTolerance(ArmConstants.j2_allE);

    j3Controller =
        new ProfiledPIDController(
            ArmConstants.j3_kP, ArmConstants.j3_kI, ArmConstants.j3_kD, j3Profile, 0.02);
    j3Controller.setTolerance(ArmConstants.j3_allE);

    j4Controller =
        new ProfiledPIDController(
            ArmConstants.j4_kP, ArmConstants.j4_kI, ArmConstants.j4_kD, j4Profile, 0.02);
    j4Controller.setTolerance(ArmConstants.j4_allE);
  }

  public double getJ1position() {
    // return J1_Encoder.getAbsolutePosition();
    return J1_Encoder.getPosition();
  }

  public double getJ2position() {
    // return J2_Encoder.getAbsolutePosition();
    return J2_Encoder.getPosition();
  }

  public double getJ3position() {
    // return J1_Encoder.getAbsolutePosition();
    return J3_Encoder.getPosition();
  }

  public double getJ4position() {
    // return J2_Encoder.getAbsolutePosition();
    return J4_Encoder.getPosition();
  }

  public void moveArm(double[] positions){

    j1Controller.setGoal(positions[0]);
    j2Controller.setGoal(positions[1]);
    j3Controller.setGoal(positions[2]);
    j4Controller.setGoal(positions[3]);

    moveJ1(calcJ1());
    moveJ2(calcJ2());
    //moveJ3(calcJ3());
    //moveJ4(calcJ4());
  }

  public void moveJ1(double speed) {
    if ((speed>0 && getJ1position() < ArmConstants.J1_Encoder_Max)||(speed<0 && getJ1position()>ArmConstants.J1_Encoder_Min)) {
      J1_motor.set(speed);
    } else {
      stopJ1();
    }
  }
  public void moveJ2(double speed) {
    if ((speed>0 && getJ2position() < ArmConstants.J2_Encoder_Max)||(speed<0 && getJ2position()>ArmConstants.J2_Encoder_Min)) {
      J2_motor.set(speed);
    } else {
      stopJ2();
    }
  }

  public void moveJ3(double speed) {
    if ((speed>0 && getJ3position() < ArmConstants.J3_Encoder_Max)||(speed<0 && getJ3position()>ArmConstants.J3_Encoder_Min)) {
      J3_motor.set(speed);
    } else {
      stopJ3();
    }
  }
  public void moveJ4(double speed) {
    if ((speed>0 && getJ4position() < ArmConstants.J4_Encoder_Max)||(speed<0 && getJ4position()>ArmConstants.J4_Encoder_Min)) {
      J4_motor.set(speed);
    } else {
      stopJ4();
    }
  }
  public void stopJ1() {
    J1_motor.set(0.0);
  }
  public void stopJ2() {
    J2_motor.set(0.0);
  }
  
  public void stopJ3() {
    J1_motor.set(0.0);
  }
  public void stopJ4() {
    J2_motor.set(0.0);
  }

  public void allStop(){
    stopJ1();
    stopJ2();
    stopJ3();
    stopJ4();
  }

  public void resetJ1Position() {
    j1Controller.reset(getJ1position());
  }

  public void resetJ2Position() {
    j2Controller.reset(getJ2position());
  }
  public void resetJ3Position() {
    j3Controller.reset(getJ3position());
  }

  public void resetJ4Position() {
    j4Controller.reset(getJ4position());
  }

  public void resetPositions(){
    resetJ1Position();
    resetJ2Position();
    resetJ3Position();
    resetJ4Position();
  }

  public double calcJ1(){
    return j1Controller.calculate(getJ1position());
  }

  public double calcJ2(){
    return j2Controller.calculate(getJ2position());
  }

  public double calcJ3(){
    return j3Controller.calculate(getJ3position());
  }

  public double calcJ4(){
    return j4Controller.calculate(getJ4position());
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("J1 Output", J1_motor.getAppliedOutput());
    SmartDashboard.putNumber("j1positionSub", getJ1position());
    SmartDashboard.putNumber("J1 Goal", j1Controller.getGoal().position);
    // This method will be called once per scheduler run
  }
}
