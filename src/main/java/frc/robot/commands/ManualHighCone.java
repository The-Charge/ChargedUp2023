// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.MagicArm;

public class ManualHighCone extends CommandBase {
  private MagicArm m_arm;
  private double m_xMultiplier;
  private boolean canMove = true;
  private long startTime = 0;
  private long m_timeOut = 0;
  /**
   * Create a dunk motion to straighten out both arms and pop down for maximum reach
   * @param subsystem
   * @param _isBackScore  back is where air tanks are
   * @param _timeOutMS
   */
  /** Creates a new ManualHighCone. */
  public ManualHighCone(MagicArm subsystem, boolean _isBackScore, long _timeOutMS) {
    if (_isBackScore) {
      m_xMultiplier = 1;
    } else {
      m_xMultiplier = -1;
    }
    m_arm = subsystem;
    m_timeOut = _timeOutMS;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // check whether it is safe to perform dunk motion
    canMove = (Math.abs(m_arm.getX() * m_xMultiplier) > ArmConstants.hiGoalX - 0.2 && m_arm.getY() > ArmConstants.hiGoalY - 0.2);
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (canMove) {
      if (Math.abs(m_arm.getElbowAngle()) < Math.PI) { //straighten out elbow
        m_arm.runElbow(-m_xMultiplier * (Math.PI + 0.4));
      } else {
        m_arm.run(m_xMultiplier * ArmConstants.shoulderScoreDegree / 180.0 * Math.PI, -m_xMultiplier * Math.PI);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!canMove || Math.abs(m_arm.getShoulderAngle()) > (ArmConstants.shoulderScoreDegree - 0.05) ||
        (System.currentTimeMillis() - startTime > m_timeOut));
  }
}