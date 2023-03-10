// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MagicArm;

public class MoveMagicArmToXY extends CommandBase {
  private double m_X, m_Y;
  private MagicArm m_arm;
  private boolean canMove = true;

  public MoveMagicArmToXY(MagicArm subsystem, double _x, double _y) {
    m_X = _x;
    m_Y = _y;
    m_arm = subsystem;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    canMove = m_arm.moveTowardXY(m_X, m_Y);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    canMove = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double[] xy = m_arm.getXY();
    return (((xy[0] - m_X) * (xy[0] - m_X) + (xy[1] - m_Y) * (xy[1] - m_Y) < 0.01) || (!canMove));
  }
}
