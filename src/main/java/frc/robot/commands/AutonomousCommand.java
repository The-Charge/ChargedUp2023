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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MagicArm;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

public class AutonomousCommand extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    private Command m_autonomousCommand;

    private double ksVolts;
    private double kvVoltSecondsPerMeter;
    private double kaVoltSecondsSquaredPerMeter;
    private double kPDriveVel;
    private double kIDriveVel;
    private double kDDriveVel;
    private double kMaxSpeedMetersPerSecond;
    private double kMaxAccelerationMetersPerSecondSquared;

    private final Drivetrain m_driveTrain;
    private final MagicArm m_arm;
    private final Claw m_claw;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    public AutonomousCommand(Drivetrain subsystem, MagicArm subsystem2, Claw subsystem3) {

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_driveTrain = subsystem;
        m_arm = subsystem2;
        m_claw = subsystem3;
        addRequirements(m_driveTrain);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        subsystem.resetEncoders();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // This checks if a path is selected
        if (RobotContainer.getInstance().getSelectedPath() != "None") {
            m_autonomousCommand = getAutonomousCommandPath();
        }

        // Schedule the autonomous command(example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    public Command getAutonomousCommandPath() {
        getSmartDashValues();

        /**
         * This will load the file "Example Path.path" and generate it with a max
         * Velocity of 4 m/s and a max acceleration of 3 m/s^2
         */
        switch (RobotContainer.getInstance().getSelectedPath()) {
            case "Charge Station With Score":
                return scoreHighConeChargeStationCommand();
            case "Charge Station No Score":
                return climbCommand();
            case "Charge Station Two Piece Left":               
                return scoreHighConeChargeStationTwoPieceCommand(-0.9);
            case "Charge Station Two Piece Right":
                return scoreHighConeChargeStationTwoPieceCommand(0.9);
            case "Charge Station Two Piece Score Left":
                return scoreHighConeChargeStationTwoPieceScoreCommand(-0.9);
            case "Charge Station Two Piece Score Right":
                return scoreHighConeChargeStationTwoPieceScoreCommand(+0.9);
            case "Charge Station Two Piece Score Balance Left":
                return scoreHighConeChargeStationTwoPieceScoreBalanceCommand(-0.9);
            case "Charge Station Two Piece Score Balance Right":
                return scoreHighConeChargeStationTwoPieceScoreBalanceCommand(0.9); 
            case "Score Cone Only":
                return scoreHighConeCommand();
            case "Clear Two Ball":
                return pathPlannerComandGroup(RobotContainer.getInstance().getSelectedPath());
            case "Bump Two Ball":
                return pathPlannerComandGroup(RobotContainer.getInstance().getSelectedPath());
            default:
                return pathPlannerComand(RobotContainer.getInstance().getSelectedPath());
        }
    }

    private void getSmartDashValues() {
        // Get Volt constraints for SysID
        ksVolts = Constants.SysIDConstants.ksVolts;
        kvVoltSecondsPerMeter = Constants.SysIDConstants.kvVoltSecondsPerMeter;
        kaVoltSecondsSquaredPerMeter = Constants.SysIDConstants.kaVoltSecondsSquaredPerMeter;

        // Get PID values for SysID
        kPDriveVel = Constants.SysIDConstants.kPDriveVel;
        kIDriveVel = Constants.SysIDConstants.kIDriveVel;
        kDDriveVel = Constants.SysIDConstants.kDDriveVel;

        // Get max speed and acceleration for SysID
        kMaxSpeedMetersPerSecond = Constants.SysIDConstants.kMaxSpeedMetersPerSecond;
        kMaxAccelerationMetersPerSecondSquared = Constants.SysIDConstants.kMaxAccelerationMetersPerSecondSquared;
    }

    public SequentialCommandGroup scoreHighConeChargeStationCommand() {
        return new SequentialCommandGroup(
                new ResetHeading(m_driveTrain),
                new ResetPitch(m_driveTrain),
                new ScoreHighCone(m_arm, true),
                new OpenClaw(m_claw, true),
                new MoveMagicArmToXY(m_arm, -0.91, 1.65, 500),
                new CloseClaw(m_claw, true),
                new ParallelCommandGroup(
                        new MoveMagicArmToXY(m_arm, -0.48, 0, 2000),
                        new DriveOver(m_driveTrain, -0.8, 10, 0)),
                new ParallelCommandGroup(
                        new MoveArmToNeutral(m_arm),
                        new SequentialCommandGroup(
                                new DriveForward(m_driveTrain, 0.8, 10, 0),
                                new Climb(m_driveTrain, 0))));
    }

    public SequentialCommandGroup climbCommand() {
        // Turns 180 after climbing over to account for potentially inactive arm
        return new SequentialCommandGroup(new ResetHeading(m_driveTrain), new ResetPitch(m_driveTrain),
                new DriveOver(m_driveTrain, -0.8, 10, 0),
                new DriveForward(m_driveTrain, 0.8, 10, 180),
                new Climb(m_driveTrain, 180));
    }

    public SequentialCommandGroup scoreHighConeCommand() {
        return new SequentialCommandGroup(new ScoreHighCone(m_arm, true),
                new OpenClaw(m_claw, true),
                new MoveArmToNeutral(m_arm),
                new CloseClaw(m_claw, true));
    }

    private SequentialCommandGroup scoreHighCone(){ 
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new ResetPitch(m_driveTrain), // resetter
                    new ResetHeading(m_driveTrain),
                    new SetStartOrientation(m_driveTrain, false)
                ),
                new ScoreHighCone(m_arm, true)
            ),
            new OpenClaw(m_claw, true) // openClaw
        );     
    }

    public SequentialCommandGroup scoreGrabScoreClar(double heading){
        return new SequentialCommandGroup(
            scoreHighCone(),
            new ParallelCommandGroup(
                new ClawSwingThroughOpen(m_claw, m_arm),
                new MoveMagicArmToXY(m_arm, ArmConstants.pickUpX, ArmConstants.pickUpY, 5000),
                new DriveDistance(m_driveTrain, -0.85, heading, Units.inchesToMeters(168))        
            ),
            new CloseClaw(m_claw, true),
            new WaitNSecs(0.5),
            new ParallelCommandGroup(
                new MoveMagicArmToXY(m_arm, -ArmConstants.hiGoalX, ArmConstants.hiGoalY - 0.05,5000), // gud
                new DriveDistance(m_driveTrain, 0.85, -Math.abs(heading)/heading*1.9, 167.5)
            ),
            new OpenClaw(m_claw, true)
        );
    }

    private SequentialCommandGroup scoreHighConeChargeStationGrab(double heading){
        return new SequentialCommandGroup(
            scoreHighCone(),
            new ParallelCommandGroup(
                new MoveMagicArmToXY(m_arm, ArmConstants.pickUpX, ArmConstants.pickUpY, 5000),
                new ClawSwingThroughOpen(m_claw, m_arm),
                new DriveOverDistance(m_driveTrain, -0.85, 10, heading, Units.inchesToMeters(30.8))
            ), 
            new CloseClaw(m_claw, true)
        );
    }

    public SequentialCommandGroup scoreHighConeChargeStationTwoPieceCommand(double headingOffset) {
        return new SequentialCommandGroup(
            scoreHighConeChargeStationGrab(headingOffset),
            new ParallelCommandGroup(
                new MoveArmToNeutral(m_arm), // gud
                new SequentialCommandGroup( // gud        
                    new DriveForward(m_driveTrain, 0.9, 10, headingOffset),
                    new Climb(m_driveTrain, headingOffset)
                )
            )
        );
    }

    public SequentialCommandGroup scoreHighConeChargeStationTwoPieceScoreCommand(double headingOffset) {
        return new SequentialCommandGroup(
            scoreHighConeChargeStationGrab(headingOffset),
            new ParallelCommandGroup(
                new MoveMagicArmToXY(m_arm, -ArmConstants.hiGoalX, ArmConstants.hiGoalY - 0.05,3000), // gud
                new DriveOver(m_driveTrain, 0.9, 10, Math.abs(headingOffset)/headingOffset*5.91)
            ),
            new OpenClaw(m_claw, true)
        );
    }
    
    public SequentialCommandGroup scoreHighConeChargeStationTwoPieceScoreBalanceCommand(double heading){
        return new SequentialCommandGroup(
            scoreHighConeChargeStationTwoPieceScoreCommand(heading),
            new ParallelCommandGroup(
                new MoveArmToNeutral(m_arm),
                new SequentialCommandGroup(
                    new DriveForward(m_driveTrain, -0.9, 10, 0), 
                    new Climb(m_driveTrain,0)
                ),
                new ClawSwingThroughOpen(m_claw, m_arm)
            )
        );
    }
    public SequentialCommandGroup pathPlannerComand(String pathName) {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath(
            pathName,
            new PathConstraints(kMaxSpeedMetersPerSecond,
                    kMaxAccelerationMetersPerSecondSquared));

        HashMap<String, Command> eventMap = new HashMap<>();

        // List of EventMap commands to be used in the Autobuilder
        eventMap.put("MoveArmScore", new ScoreHighCone(m_arm, false));
        eventMap.put("OpenClaw", new OpenClaw(m_claw, true));
        eventMap.put("MoveArmNeutral", new MoveArmToNeutral(m_arm));
        eventMap.put("MoveArmFrontLow",
                new MoveMagicArmToXY(m_arm, -ArmConstants.pickUpX, ArmConstants.pickUpY, 3000));
        eventMap.put("CloseClaw", new CloseClaw(m_claw, true));
        eventMap.put("TurnNDegrees", new TurnNRelative(m_driveTrain, 180 - m_driveTrain.getHeading()));
        eventMap.put("WaitSeconds", new WaitNSecs(10000));
        m_driveTrain.resetOdometry(new Pose2d());
        m_driveTrain.resetOdometry(new Pose2d());

        /**
         * Create the AutoBuilder. This only needs to be created once when robot code
         * Starts, not every time you want to create an auto command. A good place to
         * Put this is in RobotContainer along with your subsystems.
         */
        RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
                m_driveTrain::getPose,
                m_driveTrain::resetOdometry,
                new RamseteController(Constants.SysIDConstants.kRamseteB,
                        Constants.SysIDConstants.kRamseteZeta),
                Constants.SysIDConstants.kDriveKinematics,
                new SimpleMotorFeedforward(
                        ksVolts,
                        kvVoltSecondsPerMeter,
                        kaVoltSecondsSquaredPerMeter),
                m_driveTrain::getWheelSpeeds,
                new PIDConstants(kPDriveVel, kIDriveVel, kDDriveVel),
                m_driveTrain::tankDriveVolts,
                eventMap,
                true,
                m_driveTrain);

        Command fullAuto = autoBuilder.fullAuto(examplePath);

        // Run path following command, then stop at the end.
        return new SequentialCommandGroup(
                fullAuto.andThen(() -> m_driveTrain.tankDriveVolts(0, 0)));
    }

    public SequentialCommandGroup pathPlannerComandGroup(String pathName) {
        // Distinct method for pathgroups (Two Ball Clear/Bump)
        List<PathPlannerTrajectory> examplePath = PathPlanner.loadPathGroup(
            pathName,
            new PathConstraints(kMaxSpeedMetersPerSecond,
                    kMaxAccelerationMetersPerSecondSquared),  new PathConstraints(kMaxSpeedMetersPerSecond,
                    kMaxAccelerationMetersPerSecondSquared), new PathConstraints(kMaxSpeedMetersPerSecond,
                    kMaxAccelerationMetersPerSecondSquared));

        HashMap<String, Command> eventMap = new HashMap<>();

        // List of EventMap commands to be used in the Autobuilder
        eventMap.put("MoveArmScore", new ScoreHighCone(m_arm, false));
        eventMap.put("MoveArmScore2", new MoveMagicArmToXY(m_arm, ArmConstants.hiGoalX, ArmConstants.hiGoalY, 5000));
        eventMap.put("OpenClaw", new OpenClaw(m_claw, true));
        eventMap.put("MoveArmNeutral", new MoveArmToNeutral(m_arm));
        eventMap.put("MoveArmFrontLow",
                new MoveMagicArmToXY(m_arm, -ArmConstants.pickUpX, ArmConstants.pickUpY, 2000));
        eventMap.put("CloseClaw", new CloseClaw(m_claw, true));
        eventMap.put("TurnNDegrees", new TurnNRelative(m_driveTrain, 180 - m_driveTrain.getHeading()));
        eventMap.put("WaitSeconds", new WaitNSecs(10000));
        m_driveTrain.resetOdometry(new Pose2d());
        
        /**
         * Create the AutoBuilder. This only needs to be created once when robot code
         * Starts, not every time you want to create an auto command. A good place to
         * Put this is in RobotContainer along with your subsystems.
         */
        RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
                m_driveTrain::getPose,
                m_driveTrain::resetOdometry,
                new RamseteController(Constants.SysIDConstants.kRamseteB,
                        Constants.SysIDConstants.kRamseteZeta),
                Constants.SysIDConstants.kDriveKinematics,
                new SimpleMotorFeedforward(
                        ksVolts,
                        kvVoltSecondsPerMeter,
                        kaVoltSecondsSquaredPerMeter),
                m_driveTrain::getWheelSpeeds,
                new PIDConstants(kPDriveVel, kIDriveVel, kDDriveVel),
                m_driveTrain::tankDriveVolts,
                eventMap,
                true,
                m_driveTrain);

        Command fullAuto = autoBuilder.fullAuto(examplePath);

        // Run path following command, then stop at the end.
        return new SequentialCommandGroup(
                fullAuto.andThen(() -> m_driveTrain.tankDriveVolts(0, 0)));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
