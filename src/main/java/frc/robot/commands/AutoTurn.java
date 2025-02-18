// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoTurn extends CommandBase {
  private Drivetrain drivetrain;
  private double setPoint = 0.0;
  private double kP = 0.20;
  private double kI = 0.000;
  private double kD = 0.000;
  private PIDController controller;
  /** Creates a new AutoTurn. */
  public AutoTurn(double angle, Drivetrain subsystem) {
    drivetrain = subsystem;
    setPoint = angle;
    controller = new PIDController(kP, kI, kD);
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.enableContinuousInput(0.0, 360.0);
    controller.setSetpoint(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = controller.calculate(drivetrain.getRotation().getDegrees());
    // error = drivetrain.getYaw() - setPoint;

    // double output = kP * error;
    // SmartDashboard.putNumber("balanceout", output);
    if (output > 4.0) {
      output = 4.0;
    }
    if (output < -4.0) {
      output = -4.0;
    }

    drivetrain.drive(0.0, 0.0, output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.drive(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
