// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MagicArm;

public class MoveArmSimple extends CommandBase {
  /** Creates a new MoveArmSimple. */
  private MagicArm m_arm;
  private double targetX;
  private double targetY;
  private boolean joyStickMoved = false;
  private double deltaScale = 175.;
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
    double axDifference = RobotContainer.getInstance().getArmJoystick().getRawAxis(0);
    axDifference -= RobotContainer.getInstance().getArmJoystick().getRawAxis(2);
    if (axDifference > 1.8) deltaScale = 250;
    else if (axDifference < -1.8) deltaScale = 175;
    double xSpeed = -RobotContainer.getInstance().getArmJoystick().getRawAxis(1)/deltaScale;//.getleftJoystick().getY() / 250;
    double ySpeed = -RobotContainer.getInstance().getArmJoystick().getRawAxis(3)/deltaScale; //.getrightJoystick().getY() / 250;
    if (Math.abs(xSpeed) < (0.1/deltaScale))
      xSpeed = 0;
    else
      joyStickMoved = true;
    if (Math.abs(ySpeed) < (0.1/deltaScale))
      ySpeed = 0;
    else
      joyStickMoved = true;
    if (Math.abs(xSpeed) < (0.1/deltaScale) && Math.abs(ySpeed) < (0.1/deltaScale) && joyStickMoved) {
      double[] xy = m_arm.getXY();
      targetX = xy[0];
      targetY = xy[1];
      joyStickMoved = false;
    }
    targetX = m_arm.getLigitX(targetX + xSpeed);
    targetY = m_arm.getLigitY(targetX, targetY + ySpeed);
    m_arm.moveTowardXY(targetX, targetY);
    SmartDashboard.putNumber("targetX", targetX / 0.0254);
    SmartDashboard.putNumber("targetY", targetY / 0.0254 + 11);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
