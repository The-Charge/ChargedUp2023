// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveDegree extends CommandBase {
  /** Creates a new DriveDegree. */
  private final Drivetrain m_drivetrain;
  private final double m_heading;
  private final double m_power;
  public DriveDegree(Drivetrain subsystem, double _heading, double _power) {
    m_drivetrain = subsystem;
    m_heading = _heading;
    m_power = _power;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double fractionControl = 1.0;
    if (m_drivetrain.atHalfSpeed()) fractionControl = 0.5;
    double deltaPower = RobotContainer.getInstance().getrightJoystick().getY() / 2.0;
    double deltaHeading = RobotContainer.getInstance().getleftJoystick().getX() * 40.0 * fractionControl;
    double headingPower = (m_drivetrain.getHeading() - m_heading *fractionControl + deltaHeading) * AutoConstants.headingGain;
    double leftPower = m_power + deltaPower;
    double rightPower = leftPower - headingPower;
    leftPower += headingPower;
    double scale = Math.max(Math.max(1.0, Math.abs(leftPower)), Math.abs(rightPower));
    m_drivetrain.run(leftPower/scale, rightPower/scale);
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
