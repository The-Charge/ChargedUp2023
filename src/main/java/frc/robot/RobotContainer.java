// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  // The robot's subsystems
  public final Claw m_claw = new Claw();
  public final Drivetrain m_drivetrain = new Drivetrain();
  public final MagicArm m_arm = new MagicArm();

  // Joysticks
  private final Joystick armJoystick = new Joystick(2);
  private final Joystick rightJoystick = new Joystick(1);
  private final Joystick leftJoystick = new Joystick(0);

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Smartdashboard Subsystems
    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
    SmartDashboard.putData("ShiftLow", new ShiftLow(m_drivetrain));
    SmartDashboard.putData("ShiftHigh", new ShiftHigh(m_drivetrain));
    SmartDashboard.putData("OpenClaw", new OpenClaw(m_claw));
    SmartDashboard.putData("CloseClaw", new CloseClaw(m_claw));
    SmartDashboard.putData("ReverseDrive", new ReverseDrive(m_drivetrain));
    SmartDashboard.putData("HalfSpeed", new HalfSpeed(m_drivetrain));
    SmartDashboard.putData("QuarterSpeed", new QuarterSpeed(m_drivetrain));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
    m_drivetrain.setDefaultCommand(new TankDrive(m_drivetrain));
    m_arm.setDefaultCommand(new MoveArmSimple(m_arm));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // Configure autonomous sendable chooser
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
    // Create some buttons
    final JoystickButton halfSpeedBtn = new JoystickButton(rightJoystick, 7);
    halfSpeedBtn.onTrue(new HalfSpeed(m_drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton quarterSpeedBtn = new JoystickButton(rightJoystick, 6);
    quarterSpeedBtn.onTrue(new QuarterSpeed(m_drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton reverseDriveBtn = new JoystickButton(rightJoystick, 1);
    reverseDriveBtn.onTrue(new ReverseDrive(m_drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton closeClawBtn = new JoystickButton(rightJoystick, 12);
    closeClawBtn.onTrue(new CloseClaw(m_claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton openClawBtn = new JoystickButton(rightJoystick, 10);
    openClawBtn.onTrue(new OpenClaw(m_claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton shiftLowBtn = new JoystickButton(leftJoystick, 12);
    shiftLowBtn.onTrue(new ShiftLow(m_drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton shiftHighBtn = new JoystickButton(leftJoystick, 10);
    shiftHighBtn.onTrue(new ShiftHigh(m_drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    m_claw.setDefaultCommand(new ClawSafety(m_claw, m_arm));
    JoystickButton middleButton = new JoystickButton(rightJoystick, 2);
    middleButton.onTrue((new MoveArmToNeutral(m_arm)).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
    JoystickButton frontLowButton = new JoystickButton(rightJoystick, 3);
    frontLowButton.onTrue(
        (new MoveMagicArmToXY(m_arm, 1.0, 0.1)).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
    JoystickButton frontHighButton = new JoystickButton(rightJoystick, 5);
    frontHighButton
        .onTrue((new MoveMagicArmToXY(m_arm, 1.2, 1)).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
    JoystickButton backLowButton = new JoystickButton(rightJoystick, 4);
    backLowButton.onTrue(
        (new MoveMagicArmToXY(m_arm, -1.0, 0.1)).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
    JoystickButton backHighButton = new JoystickButton(rightJoystick, 6);
    backHighButton
        .onTrue((new MoveMagicArmToXY(m_arm, -1.2, 1)).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
    SmartDashboard.putData("AutoSelect", m_chooser);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
  }

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
  public Joystick getleftJoystick() {
    return leftJoystick;
  }

  public Joystick getrightJoystick() {
    return rightJoystick;
  }

  public Joystick getArmJoystick() {
    return armJoystick;
  }

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }

}
