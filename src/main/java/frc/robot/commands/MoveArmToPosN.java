// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class MoveArmToPosN extends CommandBase {
  private Arm m_subsystem;
  private double[] m_position;
  private double[] subdivision= {0,0,0};
  private double[] waypoint = {0,0,0};
  private int currentWaypoint = 0;
  /** Creates a new MoveJoint1. */
  public MoveArmToPosN(double[] position, Arm subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_position = position;
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetPositions(); // sets the joint controllers to their current positions
    subdivision[0] = (m_position[0] -m_subsystem.getJ1position())/5;
    subdivision[1] = (m_position[1] -m_subsystem.getJ2position())/5;
    subdivision[2] = (m_position[2] -m_subsystem.getJ3position())/5;

    waypoint[0] = m_subsystem.getJ1position()+subdivision[0];
    waypoint[1] = m_subsystem.getJ2position()+subdivision[1];
    waypoint[2] = m_subsystem.getJ3position()+subdivision[2];
    currentWaypoint = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_subsystem.moveArm(waypoint);
  
    if (m_subsystem.getAtPosition()) {
      waypoint[0] = m_subsystem.getJ1position()+subdivision[0];
      waypoint[1] = m_subsystem.getJ2position()+subdivision[1];
      waypoint[2] = m_subsystem.getJ3position()+subdivision[2];
      currentWaypoint++;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.allStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentWaypoint>5;
  }
}
