package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;

public class Climb extends CommandBase {
  private double lastPitch = 0;
  private int timesAtLevel = 0;
  private final Drivetrain m_drivetrain;
  private double m_offset = 0;

  /**
   * Traversing the charging station
   * 
   * @param subsystem     The Drivetrain subsystem used by this command.
   * @param headingOffset Angle to travel in degrees.
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
    // Power correction due to heading error.
    double headingPower = (m_drivetrain.getHeading() + m_offset) * AutoConstants.headingGain / 2;
    double power = thisPitch * AutoConstants.climbPitchGain +
        (thisPitch - lastPitch) * AutoConstants.climbPitchDerivativeGain;
    lastPitch = thisPitch;

    /*
     * Robot is not level outside of +-2 degrees of 0
     * Add bias to power depending on which direction of error and reset
     * timesAtLevel counter.
     * Otherwise, add another tick to the counter. 
     */
    if (thisPitch < -2) {
      power = power + AutoConstants.climbPowerBackwardBias;
      timesAtLevel = 0;
    } else if (thisPitch > 2) {
      power = power + AutoConstants.climbPowerForwardBias;
      timesAtLevel = 0;
    } else {
      timesAtLevel++;
    }

    // Clamp power within the speed limits (prevent sudden jerks in speed) and feed the speeds into the Drivetrain
    power = MathUtil.clamp(power, -AutoConstants.climbPowerLimit, AutoConstants.climbPowerLimit);
    m_drivetrain.run(power + headingPower, power - headingPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Ends when robot has stayed within 2 degrees of 0 for 400ms
    return (timesAtLevel > 40);
  }
}