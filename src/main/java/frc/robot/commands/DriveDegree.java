// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveDegree extends CommandBase {
  /** Creates a new DriveDegree. */
  private final Drivetrain m_drivetrain;
  private double m_heading;
  private final double m_power;

  /**
   * Field centric arcade drive with left joystick x controls the field angle and
   * right y controls power.
   * We recommend power set at 0.65 or -0.65 to use the full range of joystick and
   * usable power.
   * The robot typically stalls at around 0.3 power.
   * 
   * @param subsystem The Drivetrain subsystem used by this command.
   * @param _heading  The default field angle if no left joystick x input.
   * @param _power    The default power if no right joystick y input.
   */
  public DriveDegree(Drivetrain subsystem, double _heading, double _power) {
    m_drivetrain = subsystem;
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      m_heading = _heading;
    } else { // Blue alliane is the mirror image, default angle needs to be flipped.
      m_heading = -_heading;
    }
    if (m_drivetrain.isStarted180Off()) {
      m_heading += 180;
    }
    m_power = _power;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Speed multiplier control.
    double fractionControl = 1.0;
    if (m_drivetrain.atHalfSpeed())
      fractionControl = 0.5;
    else if (m_drivetrain.atQuarterSpeed())
      fractionControl = 0.25;

    /*
     * Full deltaPower range is -0.35 to 0.35 when added to 0.65, becomes fully
     * usable power of 0.3-1.
     * Assumes robot front side facing the driver, so that robot can climb easily
     * scoring and can use the
     * Swing to neutral after scoring to climb. In this setup, power is negtiave to
     * collect and positive.
     * To score. Push Joystick away is negative and pull towards the driver is
     * positive. The joystick
     * Matches the driver perspective.
     */
    double deltaPower = RobotContainer.getInstance().getrightJoystick().getY() * 0.35;

    /*
     * Joystick left (-x) adds counterclockwise rotation and steer to the driver's
     * left driving backwards (away from the driver).
     */
    double deltaHeading = RobotContainer.getInstance().getleftJoystick().getX() * 20.0;

    /*
     * If driving towards the driver, -x should be clockwise rotation to the left of
     * the driver.
     */
    if (m_power > 0) {
      deltaHeading = -deltaHeading;
    }
    double leftPower = m_power + deltaPower;
    double headingOffset = m_heading * fractionControl;
    if (m_drivetrain.reversed()) {
      leftPower = -leftPower;
      headingOffset += 180;
    }
    headingOffset = m_drivetrain.getHeading() - headingOffset + deltaHeading * fractionControl;
    while (headingOffset > 180)
      headingOffset -= 360;
    while (headingOffset < -180)
      headingOffset += 360;
    double headingPower = headingOffset * AutoConstants.headingGain;
    double rightPower = leftPower - headingPower;
    leftPower += headingPower;
    double scale = Math.max(Math.max(1.0, Math.abs(leftPower)), Math.abs(rightPower));
    m_drivetrain.rawRun(leftPower / scale, rightPower / scale);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_drivetrain.isIMUConnected();
  }
}
