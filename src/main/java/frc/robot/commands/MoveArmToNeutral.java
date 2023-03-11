// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.MagicArm;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class MoveArmToNeutral extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private final MagicArm m_magicArm;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


  public MoveArmToNeutral(MagicArm subsystem) {
    m_magicArm = subsystem;
    addRequirements(m_magicArm);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_magicArm.moveTowardNeutral();
  }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double[] xy = m_magicArm.getXY();
    return (Math.abs(xy[0]) < 0.02);
  }
}