// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.MagicArm;

public class ScoreHighCone extends CommandBase {
  private MagicArm m_arm;
  private double m_xMultiplier;

  /* Creates a new ScoreHighCone. */
  public ScoreHighCone(MagicArm subsystem, boolean _isScoreFront) {
    // Multiplier takes care of angle difference of front side and back side of robot
    if (_isScoreFront) {
      m_xMultiplier = 1;
    } else {
      m_xMultiplier = -1;
    }
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
    // Waits until the elbow before running shoulder
    if (Math.abs(m_arm.getElbowAngle()) < 1.7) {
      m_arm.runElbow(-m_xMultiplier * (2.8));
    } else {
      // Move arm if elbow in right position
      m_arm.run(m_xMultiplier * ArmConstants.shoulderScoreDegree / 180.0 * Math.PI, -m_xMultiplier * Math.PI);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_arm.getShoulderAngle()) > ArmConstants.shoulderScoreDegree / 180.0 * Math.PI - 0.05;
  }
}
