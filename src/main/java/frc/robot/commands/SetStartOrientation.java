// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SetStartOrientation extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Boolean facingBack;

  /**
   * Setting Orientation (for DriveDegree).
   * 
   * @param subsystem     The Drivetrain subsystem used in this command.
   * @param _backToDriver Front facing away from Driver.
   */
  public SetStartOrientation(Drivetrain subsystem, Boolean _backToDriver) {
    m_drivetrain = subsystem;
    facingBack = _backToDriver;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.start180Off(facingBack);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
