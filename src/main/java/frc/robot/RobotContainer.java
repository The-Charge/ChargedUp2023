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
  public final Claw m_claw = new Claw();
  public final Drivetrain m_drivetrain = new Drivetrain();
  public final MagicArm m_arm = new MagicArm();
  public final Camera m_camera = new Camera();

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
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Smartdashboard Subsystems
    // SmartDashboard Buttons
    // SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
    // SmartDashboard.putData("ShiftLow", new ShiftLow(m_drivetrain));
    // SmartDashboard.putData("ShiftHigh", new ShiftHigh(m_drivetrain));
    // SmartDashboard.putData("OpenClaw", new OpenClaw(m_claw));
    // SmartDashboard.putData("CloseClaw", new CloseClaw(m_claw));
    // SmartDashboard.putData("ReverseDrive", new ReverseDrive(m_drivetrain));
    // SmartDashboard.putData("HalfSpeed", new HalfSpeed(m_drivetrain));
    // SmartDashboard.putData("QuarterSpeed", new QuarterSpeed(m_drivetrain));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putData(m_claw);
    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_arm);
    // Configure default commands
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
    m_drivetrain.setDefaultCommand(new TankDrive(m_drivetrain));
    m_arm.setDefaultCommand(new MoveArmSimple(m_arm));
    m_claw.setDefaultCommand(new ClawSafety(m_claw, m_arm));
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // Configure autonomous sendable chooser
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand(m_drivetrain, m_arm, m_claw));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    pathSendableChooserInit();
  }

  private void pathSendableChooserInit() {
    // Makes new array of paths (strings)
    String[] tempPathNames = new String[(int)6];

    // Manually places in paths
    tempPathNames[0] = "None";
    tempPathNames[1] = "BottomOnePiece";
    tempPathNames[2] = "TopOnePieceEvents";
    tempPathNames[3] = "Score High Cone";
    tempPathNames[4] = "Score High Ball";
    tempPathNames[5] = "TopOnePieceEventsShort";
    // Adds path options to sendable chooser
    for (int x = 0; x < tempPathNames.length; x++) {
      m_pathSendableChooser.addOption(tempPathNames[x], tempPathNames[x]);
    }

    // Default path doesn't run the robot
    m_pathSendableChooser.setDefaultOption(tempPathNames[0], tempPathNames[0]);
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
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
    // Create some buttons
    final JoystickButton halfSpeedBtn = new JoystickButton(rightJoystick, 1);
    halfSpeedBtn.onTrue(new HalfSpeed(m_drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // final JoystickButton quarterSpeedBtn = new JoystickButton(rightJoystick, 6);
    // quarterSpeedBtn.onTrue(new QuarterSpeed(m_drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton reverseDriveBtn = new JoystickButton(leftJoystick, 1);
    reverseDriveBtn.onTrue(new ReverseDrive(m_drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // final JoystickButton turnNDegrees90Btn = new JoystickButton(rightJoystick, 10);
    // turnNDegrees90Btn
    //     .onTrue(new TurnNRelative(m_drivetrain, 90).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // final JoystickButton turnNDegrees180Btn = new JoystickButton(rightJoystick, 12);
    // turnNDegrees180Btn
    //     .onTrue(new TurnNRelative(m_drivetrain, 180).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

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
    Command armNeutral = (new MoveArmToNeutral(m_arm)).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armFrontLow = (new MoveMagicArmToXY(m_arm, ArmConstants.pickUpX, ArmConstants.pickUpY, 5000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armBackLow = (new MoveMagicArmToXY(m_arm, -ArmConstants.pickUpX, ArmConstants.pickUpY, 5000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armFrontMid = (new MoveMagicArmToXY(m_arm, ArmConstants.midGoalX, ArmConstants.midGoalY, 5000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armBackMid = (new MoveMagicArmToXY(m_arm, -ArmConstants.midGoalX, ArmConstants.midGoalY, 5000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armFrontHigh = (new MoveMagicArmToXY(m_arm, ArmConstants.hiGoalX, ArmConstants.hiGoalY, 5000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armBackHigh = (new MoveMagicArmToXY(m_arm, -ArmConstants.hiGoalX, ArmConstants.hiGoalY, 5000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armFrontStation = (new MoveMagicArmToXY(m_arm, ArmConstants.stationX, ArmConstants.stationY, 5000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command armBackStation = (new MoveMagicArmToXY(m_arm, -ArmConstants.stationX, ArmConstants.stationY, 5000))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command manualHighFront = (new ManualHighCone(m_arm, false))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    Command manualHighBack = (new ManualHighCone(m_arm, true)).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    /*
     * JoystickButton middleButton = new JoystickButton(armController, 10);
     * middleButton.onTrue(armNeutral);
     * 
     * JoystickButton frontLowButton = new JoystickButton(armController, 3);
     * frontLowButton.onTrue(armFrontLow);
     * 
     * JoystickButton frontHighButton = new JoystickButton(armController, 4);
     * frontHighButton.onTrue(armFrontHigh);
     * 
     * JoystickButton backLowButton = new JoystickButton(armController, 1);
     * backLowButton.onTrue(armBackLow);
     * 
     * JoystickButton backHighButton = new JoystickButton(armController, 2);
     * backHighButton.onTrue(armBackHigh);
     */
    SmartDashboard.putData("AutoSelect", m_chooser);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
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
    /*
    * JoystickButton backHiButton = new JoystickButton(armController, 9);
    * backHiButton.onTrue(armBackHigh);
    */
    JoystickButton frontHighButton = new JoystickButton(armController, 4);
    frontHighButton.onTrue(armFrontHigh);
    /* 
    * JoystickButton backMidButton = new JoystickButton(armController, 5);
    * backMidButton.onTrue(armBakcMid);
    */
    JoystickButton frontMidButton = new JoystickButton(armController, 1);
    frontMidButton.onTrue(armFrontMid);
    /*
    * JoystickButton backLowButton = new JoystickButton(armController, 7);
    * backLowButton.onTrue(armBackLow);
    */
    JoystickButton frontLowButton = new JoystickButton(armController, 2);
    frontLowButton.onTrue(armFrontLow);
    /*
    * JoystickButton backStationButton = new JoystickButton(armController, 1);
    * backStationButton.onTrue(armBackStation);
    */
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

    /*
     * In case it gets deleted
     * 
     * m_claw.setDefaultCommand(new ClawSafety(m_claw, m_arm));
     * 
     * JoystickButton middleButton = new JoystickButton(rightJoystick, 2);
     * middleButton.onTrue((new
     * MoveArmToNeutral(m_arm)).withInterruptBehavior(Interrupt#ionBehavior.
     * kCancelSelf));
     * 
     * JoystickButton frontLowButton = new JoystickButton(rightJoystick, 3);
     * frontLowButton.onTrue(
     * (new MoveMagicArmToXY(m_arm, 1.0,
     * 0.1)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
     * 
     * JoystickButton frontHighButton = new JoystickButton(rightJoystick, 5);
     * frontHighButton
     * .onTrue((new MoveMagicArmToXY(m_arm, 1.2,
     * 1)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
     * 
     * JoystickButton backLowButton = new JoystickButton(rightJoystick, 4);
     * backLowButton.onTrue(
     * (new MoveMagicArmToXY(m_arm, -1.0,
     * 0.1)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
     * 
     * JoystickButton backHighButton = new JoystickButton(rightJoystick, 6);
     * backHighButton
     * .onTrue((new MoveMagicArmToXY(m_arm, -1.2,
     * 1)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
     * 
     * SmartDashboard.putData("AutoSelect", m_chooser);
     * 
     */
  }

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
  public Joystick getleftJoystick() {
    return leftJoystick;
  }

  public Joystick getrightJoystick() {
    return rightJoystick;
  }

  public XboxController getArmJoystick() {
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
