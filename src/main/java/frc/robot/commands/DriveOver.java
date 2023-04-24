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
   * Glide for 1 second (50 intervals) to make sure it is out of community.
   * The command times out at 6 seconds so it will not accidentally get penalty.
   *
   * @param subsystem     The Drivetrain subsystem used in this command.
   * @param power         The initial power for climbing.
   * @param stopPitch     The pitch tht the robot is considered climbing.
   * @param headingOffset The heading the robot needs to follow (clockwise +).
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

    /*
     * Status logic
     * 0 is flat on the floor starting state,
     * 1 for driving onto the chargestation,
     * 2 for level on the charge station, 3 for driving off the charge station,
     * 4 for flat on the floor at the end.
     */
    if (status == 0) {
      if (Math.abs(thisPitch) > m_stopPitch) {
        status = 1; // Upward climbing.
      }
    } else if (status == 1) {
      if (Math.abs(thisPitch) < 4) {
        status = 2; // On top of the charge station.
        m_power = m_power * 0.8;
      }
    } else if (status == 2) {
      if (Math.abs(thisPitch) > m_stopPitch * 0.8) {
        status = 3; // Down climbing.
        m_power = m_power * .7;
      }
    }
    m_drivetrain.rawRun(m_power + headingPower, m_power - headingPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.rawRun(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (status == 3) {
      if (Math.abs(thisPitch) < 4) {
        m_drivetrain.run(m_power / 10.0, m_power / 10.0); // On the floor.
        status++;
      }
    }
    if (status > 3) {
      status++;
    }
    return !m_drivetrain.isIMUConnected() || (status > 40) || (startTimeMS + timeoutMS < System.currentTimeMillis());
  }
}