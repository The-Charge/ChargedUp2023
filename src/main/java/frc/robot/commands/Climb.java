package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
public class Climb extends CommandBase {
  private double lastPitch = 0;
  private int timesAtLevel = 0;
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_drivetrain;
  /**
   *
   * @param subsystem The subsystem used by this command.
   */

  public Climb(Drivetrain subsystem) {
    m_drivetrain = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastPitch = m_drivetrain.getPitch();
    timesAtLevel = 0;
    SmartDashboard.putBoolean("climb", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thisPitch = m_drivetrain.getPitch();
    double thisHeading = m_drivetrain.getHeading() * AutoConstants.headingGain / 2;
    double volt = thisPitch*AutoConstants.climbPitchGain + 
                  (thisPitch-lastPitch)*AutoConstants.climbPitchDerivativeGain;
    lastPitch = thisPitch;
    if(thisPitch < -1){
      volt = volt + AutoConstants.climbPowerBackwardBias;
      timesAtLevel = 0;
    }else if (thisPitch > 1){
      volt = volt + AutoConstants.climbPowerForwardBias;
      timesAtLevel = 0;
    }else timesAtLevel++;
    if(volt > AutoConstants.climbPowerLimit) volt = AutoConstants.climbPowerLimit;
    else if (volt < -AutoConstants.climbPowerLimit) volt = -AutoConstants.climbPowerLimit;
    m_drivetrain.run(volt + thisHeading, volt - thisHeading);
    SmartDashboard.putBoolean("climb", true);
    SmartDashboard.putNumber("volt", volt);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //SmartDashboard.putBoolean("climb", false);
    SmartDashboard.putNumber("timesAtLevel", timesAtLevel);
    return false;
    //(timesAtLevel > 50);
  }
}