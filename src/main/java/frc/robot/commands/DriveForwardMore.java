package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForwardMore extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;
  private double m_power;
  private double m_powerSign = 1;
  private final double m_stopPitch;
  private boolean isTickMode = false;
  private double thisPitch = 0;
  private double m_Offset = 0;
  private int imuStatus = 0;
  /**
   * Drive forward until the pitch reaches stopPitch and then drive fastClimbTicks specified in Constants
   * @param subsystem  drivetrain
   * @param power      driving power
   * @param stopPitch  pitch to start tickcounts
   * @param headingOffset robot heading in degree (clockwise +)
   */
  public DriveForwardMore(Drivetrain subsystem, double power, double stopPitch, double headingOffset) {
    m_drivetrain = subsystem;
    m_Offset = headingOffset;
    m_power = power;
    if(m_power < 0 ) m_powerSign = -1;
    m_stopPitch = stopPitch;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isTickMode = false;
    imuStatus = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    thisPitch = m_drivetrain.getPitch();
    double headingPower = (m_drivetrain.getHeading() + m_Offset) * AutoConstants.headingGain;

    if (Math.abs(thisPitch) > m_stopPitch && !isTickMode) {
      isTickMode = true;
    }
    double power = m_power;
    if (isTickMode){
      power = m_powerSign * (0.6 + 0.2*(Math.abs(thisPitch) - m_stopPitch/2)); 
    }
    if (!m_drivetrain.isIMUConnected()){
      power = 0;
      imuStatus++;
    }
    m_drivetrain.rawRun(power + headingPower, power - headingPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.rawRun(-m_powerSign*0.4, -m_powerSign*0.4);
  }

  // Returns true when ticksCount met or dropped suddenly when counting ticks (something wrong, stop to avoid penalty).
  @Override
  public boolean isFinished() {
    return imuStatus > 40 || (isTickMode && (Math.abs(thisPitch) < (m_stopPitch / 2)));
  }
}