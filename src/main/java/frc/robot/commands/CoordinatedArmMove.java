// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

public class CoordinatedArmMove extends CommandBase {
  private Arm m_subsystem;
  private double[] m_position;
  /** Creates a new MoveJoint1. */
  public CoordinatedArmMove(double[] position, Arm subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_position = position;
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetPositions(); // sets the joint controllers to their current positions
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] adjusted_positions = {0,0,0};

    adjusted_positions[0] = -m_position[0] + ArmConstants.J1_offset;
    adjusted_positions[1] = m_position[1] + ArmConstants.J2_offset + (3.168 - m_subsystem.getJ1position()); //+ 0.25*(m_subsystem.getJ1position() - m_subsystem.getJ1goal()) ;
    adjusted_positions[2] = m_position[2] + ArmConstants.J3_offset + (1.61 - m_subsystem.getJ2position())+(3.168 - m_subsystem.getJ1position());
    m_subsystem.moveArm(adjusted_positions);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.allStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
