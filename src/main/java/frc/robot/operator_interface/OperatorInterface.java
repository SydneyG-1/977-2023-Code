// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }
  public default Trigger getTestButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getCloseButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getOpenButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getSpeedOverrideR() {
    return new Trigger(() -> false);
  }
  public default Trigger getSpeedOverrideL() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveToHigh() {
    return new Trigger(() -> false);
  }
  
  public default Trigger getMoveToMid() {
    return new Trigger(() -> false);
  }
  public default Trigger getMoveToLow() {
    return new Trigger(() -> false);
  }
  public default Trigger getMoveToPickup() {
    return new Trigger(() -> false);
  }
  public default Trigger getSafetyStop() {
    return new Trigger(() -> false);
  }
  public default Trigger getOpBut8() {
    return new Trigger(() -> false);
  }
  
  public default boolean getPickupLocation() {
    return true;
  }
  
  public default boolean getGamePieceType() {
    return false;
  }
  
}
