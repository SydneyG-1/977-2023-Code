// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;

public class Arm extends SubsystemBase {

  private final TunableNumber j1Kp = new TunableNumber("J1 kP", j1_kP);
  private final TunableNumber j1Ki = new TunableNumber("J1 kI", j1_kI);
  private final TunableNumber j1Kd = new TunableNumber("J1 kD", j1_kD);
  private final TunableNumber j1maxV = new TunableNumber("J1 Max V", j1_maxV);
  private final TunableNumber j1maxAcc = new TunableNumber("J1 Max A", j1_maxAcc);

  private final TunableNumber j2Kp = new TunableNumber("J2 kP", j2_kP);
  private final TunableNumber j2Ki = new TunableNumber("J2 kI", j2_kI);
  private final TunableNumber j2Kd = new TunableNumber("J2 kD", j2_kD);
  private final TunableNumber j2maxV = new TunableNumber("J2 Max V", j2_maxV);
  private final TunableNumber j2maxAcc = new TunableNumber("J2 Max A", j2_maxAcc);

  private final TunableNumber j3Kp = new TunableNumber("J3 kP", j3_kP);
  private final TunableNumber j3Ki = new TunableNumber("J3 kI", j3_kI);
  private final TunableNumber j3Kd = new TunableNumber("J3 kD", j3_kD);
  private final TunableNumber j3maxV = new TunableNumber("J3 Max V", j3_maxV);
  private final TunableNumber j3maxAcc = new TunableNumber("J3 Max A", j3_maxAcc);

  private CANSparkMax J1_motor =
      new CANSparkMax(ArmConstants.J1_motorSparkMaxID, MotorType.kBrushed);
  private CANSparkMax J2_motor =
      new CANSparkMax(ArmConstants.J2_motorSparkMaxID, MotorType.kBrushless);
  private CANSparkMax J3_motor =
      new CANSparkMax(ArmConstants.J3_motorSparkMaxID, MotorType.kBrushless);

  private AbsoluteEncoder J1_Encoder = J1_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private AbsoluteEncoder J2_Encoder = J2_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private AbsoluteEncoder J3_Encoder = J3_motor.getAbsoluteEncoder(Type.kDutyCycle);

  private ProfiledPIDController j1Controller;
  private TrapezoidProfile.Constraints j1Profile =
      new TrapezoidProfile.Constraints(ArmConstants.j1_maxV, ArmConstants.j1_maxAcc);
  private ArmFeedforward j1ArmFeedforward;

  private ProfiledPIDController j2Controller;
  private TrapezoidProfile.Constraints j2Profile =
      new TrapezoidProfile.Constraints(ArmConstants.j2_maxV, ArmConstants.j2_maxAcc);
  private ArmFeedforward j2ArmFeedforward;

  private ProfiledPIDController j3Controller;
  private TrapezoidProfile.Constraints j3Profile =
      new TrapezoidProfile.Constraints(ArmConstants.j3_maxV, ArmConstants.j3_maxAcc);
  private ArmFeedforward j3ArmFeedforward;

  /** Creates a new Arm. */
  public Arm() {
    super();

    J1_motor.setInverted(false);
    J2_motor.setInverted(true);
    J3_motor.setInverted(false);

    J1_motor.setIdleMode(IdleMode.kBrake);
    J2_motor.setIdleMode(IdleMode.kBrake);
    J3_motor.setIdleMode(IdleMode.kBrake);

    J1_motor.enableSoftLimit(SoftLimitDirection.kForward, false);
    J1_motor.enableSoftLimit(SoftLimitDirection.kReverse, false);

    J2_motor.enableSoftLimit(SoftLimitDirection.kForward, false);
    J2_motor.enableSoftLimit(SoftLimitDirection.kReverse, false);

    J3_motor.enableSoftLimit(SoftLimitDirection.kForward, false);
    J3_motor.enableSoftLimit(SoftLimitDirection.kReverse, false);

    J1_motor.setSmartCurrentLimit(40);
    J2_motor.setSmartCurrentLimit(40);
    J3_motor.setSmartCurrentLimit(20);

    J1_Encoder.setInverted(false);
    J1_Encoder.setPositionConversionFactor(2 * Math.PI);
    J1_Encoder.setZeroOffset(0.0);

    J2_Encoder.setInverted(false);
    J2_Encoder.setPositionConversionFactor(2 * Math.PI);
    J2_Encoder.setZeroOffset(0.0);

    J3_Encoder.setInverted(false);
    J3_Encoder.setPositionConversionFactor(2 * Math.PI);
    J3_Encoder.setZeroOffset(0.0);

    /* j1Controller =
        new ProfiledPIDController(
            ArmConstants.j1_kP, ArmConstants.j1_kI, ArmConstants.j1_kD, j1Profile, 0.02);
    j1Controller.setTolerance(ArmConstants.j1_allE);
    // j1ArmFeedforward = new ArmFeedforward(ArmConstants.j1_ks, ArmConstants.j1_kg,
    // ArmConstants.j1_kv, ArmConstants.j1_ka);

    j2Controller =
        new ProfiledPIDController(
            ArmConstants.j2_kP, ArmConstants.j2_kI, ArmConstants.j2_kD, j2Profile, 0.02);
    j2Controller.setTolerance(ArmConstants.j2_allE);
    // j2ArmFeedforward = new ArmFeedforward(ArmConstants.j2_ks, ArmConstants.j2_kg,
    // ArmConstants.j2_kv, ArmConstants.j2_ka);

    j3Controller =
        new ProfiledPIDController(
            ArmConstants.j3_kP, ArmConstants.j3_kI, ArmConstants.j3_kD, j3Profile, 0.02);
    j3Controller.setTolerance(ArmConstants.j3_allE);
    // j3ArmFeedforward = new ArmFeedforward(ArmConstants.j3_ks, ArmConstants.j3_kg,
    // ArmConstants.j3_kv, ArmConstants.j3_ka);

    */
    j1Controller = new ProfiledPIDController(j1Kp.get(), j1Ki.get(), j1Kd.get(), j1Profile, 0.02);

    j1Controller.setTolerance(ArmConstants.j1_allE);
    j1ArmFeedforward =
        new ArmFeedforward(
            ArmConstants.j1_ks, ArmConstants.j1_kg, ArmConstants.j1_kv, ArmConstants.j1_ka);

    j2Controller = new ProfiledPIDController(j2Kp.get(), j2Ki.get(), j2Kd.get(), j2Profile, 0.02);
    j2Controller.setTolerance(ArmConstants.j2_allE);
    j2ArmFeedforward =
        new ArmFeedforward(
            ArmConstants.j2_ks, ArmConstants.j2_kg, ArmConstants.j2_kv, ArmConstants.j2_ka);

    j3Controller = new ProfiledPIDController(j3Kp.get(), j3Ki.get(), j3Kd.get(), j3Profile, 0.02);
    j3Controller.setTolerance(ArmConstants.j3_allE);
    j3ArmFeedforward =
        new ArmFeedforward(
            ArmConstants.j3_ks, ArmConstants.j3_kg, ArmConstants.j3_kv, ArmConstants.j3_ka);
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

  public void moveArm(double[] positions) {

    j1Controller.setGoal(positions[0]);
    j2Controller.setGoal(positions[1]);

    j3Controller.setGoal(positions[2]);

    // j3Controller.setGoal(positions[1]-ArmConstants.J2_J3_Interaction_Offset);

    moveJ1(calcJ1());
    moveJ2(calcJ2());
    moveJ3(calcJ3());
  }

  public void moveJ1(double speed) {
    if ((speed > 0 && getJ1position() < ArmConstants.J1_Encoder_Max)
        || (speed < 0 && getJ1position() > ArmConstants.J1_Encoder_Min)) {
      J1_motor.setVoltage(speed);
    } else {
      // reset the error so integral won't accumulate while at limit
      // j1Controller.reset(getJ1position());
      stopJ1();
    }
  }

  public void moveJ2(double speed) {
    if ((speed > 0 && getJ2position() < ArmConstants.J2_Encoder_Max)
        || (speed < 0 && getJ2position() > ArmConstants.J2_Encoder_Min)) {
      J2_motor.setVoltage(speed);
    } else {
      // j2Controller.reset(getJ2position());
      stopJ2();
    }
  }

  public void moveJ3(double speed) {
    if ((speed > 0 && getJ3position() < ArmConstants.J3_Encoder_Max)
        || (speed < 0 && getJ3position() > ArmConstants.J3_Encoder_Min)) {
      J3_motor.setVoltage(speed);
    } else {

      // j3Controller.reset(getJ3position());
      stopJ3();
    }
  }

  public void stopJ1() {
    J1_motor.setVoltage(0.0);
  }

  public void stopJ2() {
    J2_motor.setVoltage(0.0);
  }

  public void stopJ3() {
    J3_motor.setVoltage(0.0);
  }

  public void allStop() {
    stopJ1();
    stopJ2();
    stopJ3();
  }

  public boolean passedGoal() {
    return getAtPosition()
        || (getJ1position() < j1Controller.getGoal().position
            && getJ2position() > j2Controller.getGoal().position
            && getJ3position() > j3Controller.getGoal().position);
  }

  public boolean getAtPosition() {

    return j1Controller.atGoal() && j2Controller.atGoal() && j3Controller.atGoal();
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

  public void resetPositions() {
    resetJ1Position();
    resetJ2Position();
    resetJ3Position();
  }

  public void updateTunables() {

    if (j1Kp.hasChanged()
        || j1Ki.hasChanged()
        || j1Kd.hasChanged()
        || j2Kp.hasChanged()
        || j2Ki.hasChanged()
        || j2Kd.hasChanged()
        || j3Kp.hasChanged()
        || j3Ki.hasChanged()
        || j3Kd.hasChanged()
        || j1maxV.hasChanged()
        || j1maxAcc.hasChanged()
        || j2maxV.hasChanged()
        || j2maxAcc.hasChanged()
        || j3maxV.hasChanged()
        || j3maxAcc.hasChanged()) {
      j1Controller.setP(j1Kp.get());
      j1Controller.setI(j1Ki.get());
      j1Controller.setD(j1Kd.get());
      j1Controller.setConstraints(new TrapezoidProfile.Constraints(j1maxV.get(), j1maxAcc.get()));
      j2Controller.setP(j2Kp.get());
      j2Controller.setI(j2Ki.get());
      j2Controller.setD(j2Kd.get());
      j2Controller.setConstraints(new TrapezoidProfile.Constraints(j2maxV.get(), j2maxAcc.get()));
      j3Controller.setP(j3Kp.get());
      j3Controller.setI(j3Ki.get());
      j3Controller.setD(j3Kd.get());
      j3Controller.setConstraints(new TrapezoidProfile.Constraints(j3maxV.get(), j3maxAcc.get()));
    }
  }

  public double getJ1FF() {

    // double ff = -0.05;
    double ff = -0.33 * getJ1position() + 0.051;
    // if (ff < -0.5) {
    //   ff = 0-0.5;
    // } else {
    //    if (ff > 0.5) {
    //       ff = 0.5;
    //     }
    //   }
    // return ff;
    return 0.0;
  }

  public double getJ2FF() {
    double ff = 0.0; // 0.05;

    // SmartDashboard.putNumber("J2 FeedForward", j2Controller.calculate(getJ2position())
    // +j2ArmFeedforward.calculate(-j2Controller.getSetpoint().position +
    // 3.0,j2Controller.getSetpoint().velocity));
    // double ff = 0.27 * getJ2position() - 0.33;
    SmartDashboard.putNumber("J2MyFF", ff);
    if (ff < -0.5) {
      ff = -0.5;
    } else {
      if (ff > 0.5) {
        ff = 0.5;
      }
    }
    return ff;
  }

  public double getJ3FF() {
    double ff = 0.0;
    // double ff = -J2_motor.getAppliedOutput();
    if (ff < -0.5) {
      ff = -.50;
    } else {
      if (ff > 0.5) {
        ff = 0.5;
      }
    }
    return ff;
  }

  public double calcJ1() {

    return j1Controller.calculate(getJ1position()) + getJ1FF();
    // +j1ArmFeedforward.calculate(j1Controller.getSetpoint().position,j1Controller.getSetpoint().velocity);
  }

  public double calcJ2() {
    return j2Controller.calculate(getJ2position()) + getJ2FF();
    // return j2Controller.calculate(getJ2position())
    // +j2ArmFeedforward.calculate(j2Controller.getSetpoint().position +
    // -3.0,j2Controller.getSetpoint().velocity);
  }

  public double calcJ3() {
    return j3Controller.calculate(getJ3position()) + getJ3FF();
    // +j3ArmFeedforward.calculate(j3Controller.getSetpoint().position,j3Controller.getSetpoint().velocity);
  }

  public boolean safeToDriveFast() {
    return (getJ1position() > 3.05) && (getJ2position() < 1.77);
  }

  public double getJ1goal() {
    return j1Controller.getGoal().position;
  }

  @Override
  public void periodic() {

    updateTunables();
    SmartDashboard.putNumber("J1 Output", J1_motor.getAppliedOutput());
    SmartDashboard.putNumber("j1positionSub", getJ1position());
    SmartDashboard.putNumber("J1 Goal", j1Controller.getGoal().position);

    SmartDashboard.putNumber("J2 Output", J2_motor.getAppliedOutput());
    SmartDashboard.putNumber("j2positionSub", getJ2position());
    SmartDashboard.putNumber("J2 Goal", j2Controller.getGoal().position);

    SmartDashboard.putNumber("J3 Output", J3_motor.getAppliedOutput());
    SmartDashboard.putNumber("j3positionSub", getJ3position());
    SmartDashboard.putNumber("J3 Goal", j3Controller.getGoal().position);

    SmartDashboard.putBoolean("J1 At Goal", j1Controller.atGoal());
    SmartDashboard.putBoolean("J2 At Goal", j2Controller.atGoal());
    SmartDashboard.putBoolean("J3 At Goal", j3Controller.atGoal());
  }
}
