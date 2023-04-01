// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.MagicArm;

//Wait for claw to get inside robot to close claw, then wait for it to be outside to open.
public class ClawSwingThroughOpen extends CommandBase {
  private final Claw m_claw; 
  private final MagicArm m_arm;
  private Boolean clawOpened = false;
  private int intervalsInsideRobot = 0;
  /** Creates a new ClawSwingThroughOpen. */
  public ClawSwingThroughOpen(Claw subsystem, MagicArm _arm) {
    m_claw = subsystem;
    m_arm = _arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intervalsInsideRobot = 0;
    clawOpened = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intervalsInsideRobot == 0){
      if (m_arm.isArmTipInsideRobotX()){
        intervalsInsideRobot++;
        m_claw.closeClaw(true);;
      }
    }else{
      if (intervalsInsideRobot > 10 && !m_arm.isArmTipInsideRobotX()){
        m_claw.openClaw(true);
        clawOpened = true;
      }
      intervalsInsideRobot++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return clawOpened;
  }
}
