package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;

public class FastClimb extends CommandBase {
  private double lastPitch = 0;
  private int timesAtLevel = 0;
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;
  private double startTick = 0;
  private double endTick = 0;
  private Boolean startClimbing = false;
  private double m_offset = 0;
  private int imuStatus = 0;
  /**
   * @param subsystem The subsystem used by this command.
   */

  public FastClimb(Drivetrain subsystem, double headingOffset) {
    m_drivetrain = subsystem;
    m_offset = headingOffset;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastPitch = m_drivetrain.getPitch();
    timesAtLevel = 0;
    imuStatus = 0;
    startTick = Math.abs(m_drivetrain.getLeftEncoder());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thisPitch = m_drivetrain.getPitch();
    // power correction due to heading error
    double headingPower = (m_drivetrain.getHeading() + m_offset) * AutoConstants.headingGain / 2;
    double power = thisPitch * AutoConstants.climbPitchGain * 1.1 +
        (thisPitch - lastPitch) * AutoConstants.climbPitchDerivativeGain;
    lastPitch = thisPitch;
    if (Math.abs(thisPitch) < 8 ){
      if (startClimbing && endTick < 1){
        endTick = Math.abs(m_drivetrain.getLeftEncoder());
      }
    }else if (!startClimbing){
      startClimbing = true;
    }
    if (thisPitch < -2.5) {
      power = power + AutoConstants.climbPowerBackwardBias;
      timesAtLevel = 0;
    } else if (thisPitch > 2.5) {
      power = power + AutoConstants.climbPowerForwardBias;
      timesAtLevel = 0;
    } else {
      timesAtLevel++;
    }
    double currentTick = Math.abs(m_drivetrain.getLeftEncoder());
    SmartDashboard.putNumber("start tick", startTick);
    SmartDashboard.putNumber("end tick", endTick);
    SmartDashboard.putNumber("current tick", currentTick);
    if(endTick < 1 || (currentTick-endTick < (startTick - endTick)/2)){
      power = MathUtil.clamp(power, -AutoConstants.climbPowerLimit-0.05, AutoConstants.climbPowerLimit+0.05);
    }else{
      power = 0;
    }
    if (!m_drivetrain.isIMUConnected()){
      power = 0;
      imuStatus++;
    }
    m_drivetrain.run(power + headingPower, power - headingPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return imuStatus > 40 || (timesAtLevel > 40);
  }
}