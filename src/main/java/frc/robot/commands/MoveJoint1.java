// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class MoveJoint1 extends CommandBase {
  private Arm m_subsystem;
  private double m_position1;
  private double m_position2;
  /** Creates a new MoveJoint1. */
  public MoveJoint1(double position1, double position2, Arm subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_position1 = position1;
    
    m_position2 = position2;
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetJ1Position();
    m_subsystem.resetJ2Position();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.moveJ1(m_position1);
    m_subsystem.moveJ2(m_position2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopJ1();
    m_subsystem.stopJ2();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
