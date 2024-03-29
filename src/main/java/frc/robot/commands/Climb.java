package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;

public class Climb extends CommandBase {
  private double lastPitch = 0;
  private int timesAtLevel = 0;
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;

  private double m_offset = 0;

  /**
   * @param subsystem The subsystem used by this command.
   */

  public Climb(Drivetrain subsystem, double headingOffset) {
    m_drivetrain = subsystem;
    m_offset = headingOffset;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastPitch = m_drivetrain.getPitch();
    timesAtLevel = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thisPitch = m_drivetrain.getPitch();

    double thisHeading = (m_drivetrain.getHeading() + m_offset) * AutoConstants.headingGain / 2;

    double volt = thisPitch * AutoConstants.climbPitchGain +
        (thisPitch - lastPitch) * AutoConstants.climbPitchDerivativeGain;

    lastPitch = thisPitch;

    if (thisPitch < -2.5) {
      volt = volt + AutoConstants.climbPowerBackwardBias;
      timesAtLevel = 0;
    } else if (thisPitch > 2.5) {
      volt = volt + AutoConstants.climbPowerForwardBias;
      timesAtLevel = 0;
    } else {
      timesAtLevel++;
    }

    volt = MathUtil.clamp(volt, -AutoConstants.climbPowerLimit, AutoConstants.climbPowerLimit);
    m_drivetrain.run(volt + thisHeading, volt - thisHeading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timesAtLevel > 40);
  }
}