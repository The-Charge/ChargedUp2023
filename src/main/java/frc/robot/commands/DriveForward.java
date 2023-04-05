package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForward extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;
  private double m_power;
  private final double m_stopPitch;
  private double startTick = 0;
  private boolean isTickMode = false;
  private double thisPitch = 0;
  private double m_Offset = 0;
  /**
   * Drive forward until the pitch reaches stopPitch and then drive fastClimbTicks specified in Constants
   * @param subsystem  drivetrain
   * @param power      driving power
   * @param stopPitch  pitch to start tickcounts
   * @param headingOffset robot heading in degree (clockwise +)
   */
  public DriveForward(Drivetrain subsystem, double power, double stopPitch, double headingOffset) {
    m_drivetrain = subsystem;
    m_Offset = headingOffset;
    m_power = power;
    m_stopPitch = stopPitch;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isTickMode = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    thisPitch = m_drivetrain.getPitch();
    double headingPower = (m_drivetrain.getHeading() + m_Offset) * AutoConstants.headingGain;

    if (Math.abs(thisPitch) > m_stopPitch && !isTickMode) {
      startTick = m_drivetrain.getLeftEncoder();
      isTickMode = true;
    }
    m_drivetrain.rawRun(m_power + headingPower, m_power - headingPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.rawRun(0, 0);
  }

  // Returns true when ticksCount met or dropped suddenly when counting ticks (something wrong, stop to avoid penalty).
  @Override
  public boolean isFinished() {
    return !m_drivetrain.isIMUConnected() || (isTickMode &&
        ((Math.abs(m_drivetrain.getLeftEncoder() - startTick) > AutoConstants.fastClimbTicks)
            || (Math.abs(thisPitch) < Math.abs(m_stopPitch) / 2)));
  }
}