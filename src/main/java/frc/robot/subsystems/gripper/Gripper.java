// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  private CANSparkMax intake_motor =
      new CANSparkMax(GripperConstants.gripperSparkMaxID, MotorType.kBrushed);
      
  private CANSparkMax intake_motor2 =
  new CANSparkMax(5, MotorType.kBrushed);
  //private PneumaticHub ph = new PneumaticHub(20);
  //private Solenoid grip = ph.makeSolenoid(1);
  private Timer time = new Timer();
  private boolean distBool = false;

  // private ColorSensorV3 distanceSensor = new ColorSensorV3(I2C.Port.kOnboard);
  //private ColorSensorV3 distanceSensor = new ColorSensorV3(I2C.Port.kMXP);

  /** Creates a new Gripper. */
  public Gripper() {
    time.start();
  }

  public void closegrip() {
    //if (!getDistanceTrigger()) {
      intake_motor.set(-GripperConstants.intakeSpeed);
      intake_motor2.set(-GripperConstants.intakeSpeed);
   // }
    //grip.set(true);
  }

  public void opengrip() {
    //grip.set(true);
  }

  public void setGrip(boolean state) {
    //grip.set(state);
  }

  public void intakeCube() {
   // if (!getDistanceTrigger()) {
      intake_motor.set(-GripperConstants.intakeSpeedCube);
      intake_motor2.set(-GripperConstants.intakeSpeedCube);
   // }
    //grip.set(false);
  }


  public void intakeCone() {
    // if (!getDistanceTrigger()) {
       intake_motor.set(-GripperConstants.intakeSpeedCone);
       intake_motor2.set(-GripperConstants.intakeSpeedCone);
    // }
     //grip.set(false);
   }

  public void releaseCube() {
    intake_motor.set(GripperConstants.releaseSpeed);
    intake_motor2.set(GripperConstants.intakeSpeed);
  }

  public void newRelease() {
    intake_motor.set(GripperConstants.releaseSpeed);
    intake_motor2.set(GripperConstants.releaseSpeed);
  }
  public void newIntake() {
    intake_motor.set(-GripperConstants.intakeSpeed);
    intake_motor2.set(-GripperConstants.intakeSpeed);
  }

  public void newIdleIntake(){
    intake_motor.set(-0.1);
    intake_motor2.set(-0.1);
  }

  public void stopIntake() {
    intake_motor.set(0.0);
    intake_motor2.set(0.0);
  }
/*
  public boolean getDistanceTrigger() {
    if (time.advanceIfElapsed(0.2)) {
      distBool = distanceSensor.getProximity() > GripperConstants.distanceValue;
      SmartDashboard.putNumber("Distance", distanceSensor.getProximity());
    }

    return (distBool);
  }*/

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Distance", distanceSensor.getProximity());
    //SmartDashboard.putBoolean("Gripper", getDistanceTrigger());
    // This method will be called once per scheduler run
  }
}
