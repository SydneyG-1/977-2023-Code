// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  private CANSparkMax intake_motor =
      new CANSparkMax(GripperConstants.gripperSparkMaxID, MotorType.kBrushed);
  private PneumaticHub ph = new PneumaticHub(20);
  private Solenoid grip = ph.makeSolenoid(1);
  /** Creates a new Gripper. */
  public Gripper() {}

  public void closegrip() {
    grip.set(true);
  }

  public void opengrip() {
    grip.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
