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

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.*;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * Should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the
 * Robot (including subsystems, commands, and button mappings) should
 * Be declared here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  // The robot's subsystems
  public final Camera m_camera = new Camera();
  public final MagicArm m_magicArm = new MagicArm();
  public final Claw m_claw = new Claw();
  public final Drivetrain m_drivetrain = new Drivetrain();

  // Joysticks
  private final XboxController armController = new XboxController(2);
  private final Joystick rightJoystick = new Joystick(1);
  private final Joystick leftJoystick = new Joystick(0);

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<String> m_pathSendableChooser = new SendableChooser<>();

  // The container for the robot. Contains subsystems, OI devices, and commands.
  private RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
    m_magicArm.setDefaultCommand(new MoveArmSimple(m_magicArm));
    m_drivetrain.setDefaultCommand(new TankDrive(m_drivetrain));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // This command has been set outside of the autogenerated code because
    // robot builder cannot handle two subsystems inside of a command
    m_claw.setDefaultCommand(new ClawSafety(m_claw, m_magicArm));

    // Configure autonomous sendable chooser
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand(m_drivetrain, m_magicArm, m_claw));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    pathSendableChooserInit();
  }

  private void pathSendableChooserInit() {
    // Makes new array of paths (strings)
    String[] pathFileNames = new String[Constants.SysIDConstants.NUMBER_OF_PATHWAYS];

    // Manually places in paths
    pathFileNames[0] = "None";
    pathFileNames[1] = "Clear";
    pathFileNames[2] = "Clear No Events";
    pathFileNames[3] = "Charge Station with Score";
    pathFileNames[4] = "Charge Station";
    pathFileNames[5] = "Bump";
    pathFileNames[6] = "Bump No Events";
    pathFileNames[7] = "Score Cone Only";
    pathFileNames[8] = "Forward 2M";

    // Adds path options to sendable chooser
    for (int x = 0; x < pathFileNames.length; x++) {
      m_pathSendableChooser.addOption(pathFileNames[x], pathFileNames[x]);
    }

    // Default path doesn't run the robot
    m_pathSendableChooser.setDefaultOption(pathFileNames[0], pathFileNames[0]);
    SmartDashboard.putData("Auto Path Chooser", m_pathSendableChooser);
  }

  public String getSelectedPath() {
    return m_pathSendableChooser.getSelected();
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * Created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
   * Then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton halfSpeedBtn = new JoystickButton(rightJoystick, 1);
    halfSpeedBtn.onTrue(new HalfSpeed(m_drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    final JoystickButton collectBtn = new JoystickButton(rightJoystick, 2);
    collectBtn.whileTrue(new DriveDegree(m_drivetrain, -20, -0.5));
    final JoystickButton scoreBtn = new JoystickButton(leftJoystick, 2);
    scoreBtn.whileTrue(new DriveDegree(m_drivetrain, 0, 0.5));


    // final JoystickButton quarterSpeedBtn = new JoystickButton(rightJoystick, 6);
    // quarterSpeedBtn.onTrue(new
    // QuarterSpeed(m_drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton reverseDriveBtn = new JoystickButton(leftJoystick, 1);
    reverseDriveBtn.onTrue(new ReverseDrive(m_drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // final JoystickButton turnNDegrees90Btn = new JoystickButton(rightJoystick,
    // 10);
    // turnNDegrees90Btn
    // .onTrue(new TurnNRelative(m_drivetrain,
    // 90).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // final JoystickButton turnNDegrees180Btn = new JoystickButton(rightJoystick,
    // 12);
    // turnNDegrees180Btn
    // .onTrue(new TurnNRelative(m_drivetrain,
    // 180).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton shiftLowBtn = new JoystickButton(rightJoystick, 4);
    shiftLowBtn.onTrue(new ShiftLow(m_drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton shiftHighBtn = new JoystickButton(rightJoystick, 6);
    shiftHighBtn.onTrue(new ShiftHigh(m_drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // final JoystickButton alignBtn = new JoystickButton(leftJoystick, 1);
    // alignBtn.onTrue(new Align(m_drivetrain, m_camera));

    // Claw Buttons
    final JoystickButton closeClawBtn = new JoystickButton(armController, 5); // was 5
    closeClawBtn.onTrue(new CloseClaw(m_claw, false).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    final JoystickButton openClawBtn = new JoystickButton(armController, 6); // was 6
    openClawBtn.onTrue(new OpenClaw(m_claw, false).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // Arm State Buttons
    Command armNeutral = (new MoveArmToNeutral(m_magicArm)).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armFrontLow = (new MoveMagicArmToXY(m_magicArm, ArmConstants.pickUpX, ArmConstants.pickUpY, 8000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armBackLow = (new MoveMagicArmToXY(m_magicArm, -ArmConstants.pickUpX, ArmConstants.pickUpY, 8000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armFrontMid = (new MoveMagicArmToXY(m_magicArm, ArmConstants.midGoalX, ArmConstants.midGoalY, 8000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armBackMid = (new MoveMagicArmToXY(m_magicArm, -ArmConstants.midGoalX, ArmConstants.midGoalY, 8000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armFrontHigh = (new MoveMagicArmToXY(m_magicArm, ArmConstants.hiGoalX, ArmConstants.hiGoalY, 8000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armBackHigh = (new MoveMagicArmToXY(m_magicArm, -ArmConstants.hiGoalX, ArmConstants.hiGoalY, 8000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armFrontStation = (new MoveMagicArmToXY(m_magicArm, ArmConstants.stationX, ArmConstants.stationY, 8000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armBackStation = (new MoveMagicArmToXY(m_magicArm, -ArmConstants.stationX, ArmConstants.stationY, 8000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command manualHighFront = (new ManualHighCone(m_magicArm, false, 3000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command manualHighBack = (new ManualHighCone(m_magicArm, true, 3000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);

    /*
     * Suggestion for xcontroller button mapping in case buttonbox is not ready
     * #9 backhigh, #10 fronthigh
     * #5 backmiddle, #6 frontmiddle
     * #7 backlow, #8 front low
     * #1 back collect station, #3 front collect station
     * #2 claw open, #4 claw close
     * #11, #12 arm neutral
     */
    JoystickButton middleButton1 = new JoystickButton(armController, 10);
    middleButton1.onTrue(armNeutral);
    JoystickButton middleButton2 = new JoystickButton(armController, 9);
    middleButton2.onTrue(armNeutral);

    JoystickButton frontHighButton = new JoystickButton(armController, 4);
    frontHighButton.onTrue(armFrontHigh);

    JoystickButton frontMidButton = new JoystickButton(armController, 1);
    frontMidButton.onTrue(armFrontMid);

    JoystickButton frontLowButton = new JoystickButton(armController, 2);
    frontLowButton.onTrue(armFrontLow);

    JoystickButton frontStationButton = new JoystickButton(armController, 3);
    frontStationButton.onTrue(armFrontStation);

    JoystickButton backScore = new JoystickButton(armController, 7);
    backScore.onTrue(manualHighBack);

    JoystickButton frontScore = new JoystickButton(armController, 8);
    frontScore.onTrue(manualHighFront);

    POVButton armBackHighButton = new POVButton(armController, 0);
    armBackHighButton.onTrue(armBackHigh);

    POVButton armBackLowButton = new POVButton(armController, 180);
    armBackLowButton.onTrue(armBackLow);

    POVButton armBackMidButton = new POVButton(armController, 270);
    armBackMidButton.onTrue(armBackMid);

    POVButton armBackCollectButton = new POVButton(armController, 90);
    armBackCollectButton.onTrue(armBackStation);
  }

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
  public Joystick getleftJoystick() {
    return leftJoystick;
  }

  public Joystick getrightJoystick() {
    return rightJoystick;
  }

  public XboxController getArmController() {
    return armController;
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
