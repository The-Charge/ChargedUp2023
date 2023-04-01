// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MagicArm;

public class MidGoalDepth extends CommandBase {
  private final Drivetrain m_driveTrain;
  private final MagicArm m_arm;
  private final double armX;
  private final double armY;
  /** Creates a new MidGoalDepth. */
  /**
   * Move armtip so that it is the same depth as when the robot is straight against the goal
   * @param subsystem arm
   * @param gyroTrain drivetrain for heading input
   * @param _x
   * @param _y
   */
  public MidGoalDepth(MagicArm subsystem, Drivetrain gyroTrain, double _x, double _y) {
    m_driveTrain = gyroTrain;
    m_arm = subsystem;
    armX = _x;
    armY = _y;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.moveTowardXY(armX, armY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double cosine = Math.abs(Math.cos(Units.degreesToRadians(m_driveTrain.getHeading())));
    m_arm.moveTowardXY(armX/cosine, armY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when joysticks on gamepad moved.
  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.getInstance().getArmController().getRawAxis(1)) > 0.1
        || Math.abs(RobotContainer.getInstance().getArmController().getRawAxis(3)) > 0.1;
  }
}
