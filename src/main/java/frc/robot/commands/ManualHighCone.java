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
  private long m_timeOut;

  /** Creates a new ManualHighCone. */
  public ManualHighCone(MagicArm subsystem, boolean _isBackScore, long _timeOutMs) {
    if(_isBackScore) m_xMultiplier = 1;
    else m_xMultiplier = -1;
    m_arm = subsystem;
    addRequirements(m_arm);
    m_timeOut = _timeOutMs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double[] xy = m_arm.getXY();
    canMove = (Math.abs(xy[0]) > ArmConstants.hiGoalX - 0.1 && xy[1] > ArmConstants.hiGoalY-0.1);
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (canMove) {
      if(Math.abs(m_arm.getElbowAngle()) < Math.PI)m_arm.runElbow(-m_xMultiplier*(Math.PI+0.4));
      else{
        m_arm.run(m_xMultiplier*ArmConstants.shoulderScoreDegree/180.0*Math.PI, -m_xMultiplier*Math.PI);
        canMove = false;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_arm.getShoulderAngle()) > ArmConstants.shoulderScoreDegree - .05){
      return true;
    } else if((System.currentTimeMillis() - startTime) > m_timeOut){
      return true;
    } return false;
  }
}
