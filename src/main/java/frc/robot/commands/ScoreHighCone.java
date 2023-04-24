// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.MagicArm;

public class ScoreHighCone extends CommandBase {
  private MagicArm m_arm;
  private final double m_xMultiplier;

  /**
   * MoveArm to "High" scoring position and OpenClaw (for autonomous paths only).
   * 
   * @param subsystem     The MagicArm subsystem used in this command.
   * @param _isScoreFront Score on front side of robot or backside of robot.
   */
  public ScoreHighCone(MagicArm subsystem, boolean _isScoreFront) {
    // Account for front score or back score.
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
    if (Math.abs(m_arm.getElbowAngle()) < 1.1) {
      // Run the elbow first to get to position where shoulder can move without hitting high node pole.
      m_arm.runElbow(-m_xMultiplier * (2.8));
    } else {
      // Run both parts of the arm to reach setpoint.
      m_arm.run(m_xMultiplier * (ArmConstants.shoulderScoreDegree + 5) / 180.0 * Math.PI, -m_xMultiplier * (Math.PI + 0.05));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Command ends when shoulder exceeds the degrees necessary for scoring.
    return Math.abs(m_arm.getShoulderAngle()) > ArmConstants.shoulderScoreDegree * Constants.degreeToRadian - 0.06;
  }
}
