// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.gripper.Gripper;

public class AutoMove1 extends CommandBase {

  private Drivetrain drivetrain;
  private Gripper gripper;
  /** Creates a new AutoMove1. */
  public AutoMove1(Drivetrain subsystem, Gripper subsystem2) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = subsystem;
    gripper = subsystem2;
    addRequirements(drivetrain, gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // drivetrain.resetOdometry(null);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gripper.releaseCube();
    drivetrain.drive(1.2, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0.0, 0.0, 0.0);
    gripper.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
