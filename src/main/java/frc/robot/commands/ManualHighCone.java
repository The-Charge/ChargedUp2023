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
    double[] xy = m_arm.getXY();
    canMove = (Math.abs(xy[0]) > ArmConstants.hiGoalX - 0.2 && xy[1] > ArmConstants.hiGoalY - 0.2);
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (canMove) {
      if (Math.abs(m_arm.getElbowAngle()) < Math.PI - 0.1) {
        m_arm.runElbow(-m_xMultiplier * (Math.PI + 0.2));
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
    return (!canMove || Math.abs(m_arm.getShoulderAngle()) > (ArmConstants.shoulderScoreDegree - 0.04) ||
        (System.currentTimeMillis() - startTime > m_timeOut));
  }
}
