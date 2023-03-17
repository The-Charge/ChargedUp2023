package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveOver extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;
  private double m_speed;
  private final double m_stopPitch;
  private double thisPitch;
  private double m_offset = 0;
  private int status = 0;

  /**
   * 0 is flat starting state, 1 for up, 2 for flat top, 3 for down, 4 for flat
   * End.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveOver(Drivetrain subsystem, double speed, double stopPitch, double headingOffset) {
    m_drivetrain = subsystem;
    m_offset = headingOffset;
    m_speed = speed;
    m_stopPitch = stopPitch;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    status = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    thisPitch = m_drivetrain.getPitch();
    double thisHeading = (m_drivetrain.getHeading() + m_offset) * AutoConstants.headingGain;
    if (status == 0) {
      if (Math.abs(thisPitch) > m_stopPitch) {
        status = 1;
      }
    } else if (status == 1) {
      if (Math.abs(thisPitch) < 2) {
        status = 2;
        m_speed = m_speed * 0.8;
      }
    } else if (status == 2) {
      if (Math.abs(thisPitch) > m_stopPitch * 0.8) {
        status = 3;
        m_speed = m_speed * .7;
      }
    }
    m_drivetrain.run(m_speed + thisHeading, m_speed - thisHeading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.run(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (status == 3) {
      if (Math.abs(thisPitch) < 1) {
        m_drivetrain.run(m_speed/100, m_speed/100); //0, 0);
        status++;
      }
    }
    if (status > 3) {
      status++;
    }
    if (status > 20) {
      return true;
    }
    return false;
  }
}