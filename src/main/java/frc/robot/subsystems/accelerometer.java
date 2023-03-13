// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class accelerometer extends SubsystemBase {
  /** Creates a new accelerometer. */
  private Accelerometer accelerometer = null;
  //private Accelerometer accelerometer = new BuiltInAccelerometer();
  public accelerometer() {  
    
  }


  public double getAccelValue(){
    return 2.5;
    
    //return accelerometer.getZ();
  }
  
  @Override
  public void periodic() {

    
    //SmartDashboard.putNumber("Acceleration", getAccelValue());
    // This method will be called once per scheduler run
  }
}
