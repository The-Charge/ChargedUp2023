// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveDegree extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final double m_power;
  private final double m_heading;
  
  /** Creates a new DriveDegree. */
  
  public DriveDegree(Drivetrain subsystem, double _heading, double _power) {
    m_drivetrain = subsystem;
    addRequirements(subsystem);
    m_heading = _heading;
    m_power = _power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double deltaPower = RobotContainer.getInstance().getrightJoystick().getY() / 2;
    double deltaHeading =  RobotContainer.getInstance().getleftJoystick().getX() * 40;
    if (m_power > 0) deltaHeading = -deltaHeading;
    double headingAdjustment = (m_drivetrain.getHeading() - m_heading + deltaHeading) * AutoConstants.headingGain;
    double leftPower = m_power + deltaPower;
    double rightPower = leftPower - headingAdjustment;
    leftPower += headingAdjustment;
    double scaleFactor = Math.max(Math.max(1.0,Math.abs(leftPower)), Math.abs(rightPower));
    m_drivetrain.run(leftPower / scaleFactor, rightPower / scaleFactor);
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
