// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveDegree extends CommandBase {
  /** Creates a new DriveDegree. */
  private final Drivetrain m_drivetrain;
  private final double m_heading;
  private final double m_power;
  /**
   * Field centric arcade drive with left joystick x controls the field angle and right y controls power
   * We recommend power set at 0.65 or -0.65 to use the full range of joystick and usable power
   * The robot typically stalls at around 0.3 power
   * @param subsystem drivetrain
   * @param _heading  the default field angle if no left joystick x input
   * @param _power  the default power if no right joystick y input
   */
  public DriveDegree(Drivetrain subsystem, double _heading, double _power) {
    m_drivetrain = subsystem;
    if(SmartDashboard.getBoolean("Red Alliance", true)){
      m_heading = _heading;
    }else{  // Blue alliane is the mirror image, default angle needs to be fliped
      m_heading = -_heading;
    }
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
    // Full deltaPower range is -0.35 to 0.35 when added to 0.65, becomes fully usable power of 0.3-1
    // Assumes robot front side facing the driver, so that robot can climb easily scoring and can use the
    // Swing to neutral after scoring to climb.  In this setup, power is negtiave to collect and positive 
    // To score.  Push Joystick away is negative and pull towards the driver is positive.  The joystick
    // Matches the driver perspective.
    double deltaPower = RobotContainer.getInstance().getrightJoystick().getY() * 0.35;
    // Joystick left (-x) adds counterclockwise rotation and steer to the driver's left driving backwards (away from the driver). 
    double deltaHeading = RobotContainer.getInstance().getleftJoystick().getX() * 20.0;
    // If driving towards the driver, -x should be clockwise rotation to the left of the driver 
    if (m_power > 0) {
      deltaHeading = -deltaHeading;
    }
    double headingPower = (m_drivetrain.getHeading() - (m_heading - deltaHeading) * fractionControl) * AutoConstants.headingGain;
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
