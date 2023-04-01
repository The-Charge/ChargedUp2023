package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveOver extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;
  private double m_power;
  private final double m_stopPitch;
  private double thisPitch;
  private double m_offset = 0;
  private int status = 0;
  public double timeoutMS = 6000;
  public long startTimeMS;

  /**
   * Drive over the charging station.
   * status 0 is flat starting state, 1 for up climing, 2 for flat top charging station, 3 for down climbing, 
   * 4 for flat end.  Glide for 1 second (50 intervals) to make sure it is out of community.  
   * The command times out at 6 seconds so it will not accidentally get penalty.
   *
   * @param subsystem drivetrain.
   * @param power the initial power for climbing
   * @param stopPitch the pitch tht the robot is considered climbing
   * @param headingOffset the heading the robot needs to follow (clockwise +)
   */
  public DriveOver(Drivetrain subsystem, double power, double stopPitch, double headingOffset) {
    m_drivetrain = subsystem;
    m_offset = headingOffset;
    m_power = power;
    m_stopPitch = stopPitch;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    status = 0;
    startTimeMS = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    thisPitch = m_drivetrain.getPitch();
    double headingPower = (m_drivetrain.getHeading() + m_offset) * AutoConstants.headingGain;

    if (status == 0) {
      if (Math.abs(thisPitch) > m_stopPitch) {
        status = 1; // we are climbing
      }
    } else if (status == 1) {
      if (Math.abs(thisPitch) < 4) {
        status = 2; // we are on top of the charge station
        m_power = m_power * 0.8;
      }
    } else if (status == 2) {
      if (Math.abs(thisPitch) > m_stopPitch * 0.8) {
        status = 3;  // we are down climbing
        m_power = m_power * .7;
      }
    }
    m_drivetrain.run(m_power + headingPower, m_power - headingPower);
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
      if (Math.abs(thisPitch) < 4) {
        m_drivetrain.run(m_power / 10.0, m_power / 10.0); // we are flat again
        status++;
      }
    }
    if (status > 3) {
      status++;
    }
    return (status > 50) || (startTimeMS + timeoutMS < System.currentTimeMillis());
  }
}