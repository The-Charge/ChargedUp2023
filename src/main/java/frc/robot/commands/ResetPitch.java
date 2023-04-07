package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ResetPitch extends CommandBase {
  private final Drivetrain m_drivetrain;

  /**
   * Reset the NavX pitch (at the start of non-pathplanner autonomous paths)
   * 
   * @param subsystem  Drivetrain subsystem
   */
  public ResetPitch(Drivetrain subsystem) {
    m_drivetrain = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetPitch();
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