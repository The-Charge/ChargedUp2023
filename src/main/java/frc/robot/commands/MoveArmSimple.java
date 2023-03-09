// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.MagicArm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveArmSimple extends CommandBase {
  /** Creates a new MoveArmSimple. */
  private MagicArm m_arm;
  private double targetX;
  private double targetY;
  private boolean joyStickMoved = false;
  public MoveArmSimple(MagicArm subsystem) {
    m_arm = subsystem;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double[] xy = m_arm.getXY();
    targetX = xy[0];
    targetY = xy[1];
    joyStickMoved = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -RobotContainer.getInstance().getleftJoystick().getY()/500;
    double ySpeed = -RobotContainer.getInstance().getrightJoystick().getY()/500;
    if (Math.abs(xSpeed) < 0.0005)xSpeed = 0; else joyStickMoved = true;
    if (Math.abs(ySpeed) < 0.0005)ySpeed = 0; else joyStickMoved = true;
    if(Math.abs(xSpeed) < 0.0005 && Math.abs(ySpeed) <0.0005 && joyStickMoved){
      double[] xy = m_arm.getXY(); 
      targetX = xy[0];
      targetY = xy[1];
      joyStickMoved = false;
    }      
    double oldX = targetX;
    double oldY = targetY;
    targetX = targetX + xSpeed;
    targetY = targetY + ySpeed;
    if(!m_arm.isXYInLimit(targetX, targetY)){
      targetX = oldX;
      targetY = oldY;
    }
    SmartDashboard.putNumber("targetX", targetX);
    SmartDashboard.putNumber("targetY", targetY);
    m_arm.moveToXY(targetX, targetY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
