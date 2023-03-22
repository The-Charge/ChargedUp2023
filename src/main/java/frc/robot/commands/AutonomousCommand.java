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

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
            case "Charge Station with Score":
                return scoreHighConeChargeStationCommand();
            case "Charge Station":
                return climbCommand();
            case "Score Cone Only":
                return scoreHighConeCommand();
            default:
                return pathPlannerComand();
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
                new MoveMagicArmToXY(m_arm, -0.91, 1.65, 200),
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
        return new SequentialCommandGroup(new ScoreHighCone(m_arm, true));
    }

    public SequentialCommandGroup pathPlannerComand() {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath(
                RobotContainer.getInstance().getSelectedPath(),
                new PathConstraints(kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared));

        HashMap<String, Command> eventMap = new HashMap<>();
        // Scores Current game piece
        eventMap.put("MoveArmScore", new ScoreHighCone(m_arm, false));
        eventMap.put("OpenClaw", new OpenClaw(m_claw, true));
        eventMap.put("MoveArmIntermediate", new MoveMagicArmToXY(m_arm, 0.91, 1.65, 500));
        /**
         * Centers gravity for accurate autonomous pathing and
         * Makes next event quicker
         */
        eventMap.put("MoveArmNeutral", new MoveArmToNeutral(m_arm));

        // Picks up game piece
        eventMap.put("MoveArmFrontLow",
                new MoveMagicArmToXY(m_arm, -ArmConstants.pickUpX, ArmConstants.pickUpY, 8000));
        eventMap.put("CloseClaw", new CloseClaw(m_claw, true));

        /**
         * Centers gravity for accurate autonomous pathing and
         * Makes next event quicker again
         */
        eventMap.put("MoveArmNeutral", new MoveArmToNeutral(m_arm));

        // Scores previously grabbed piece
        eventMap.put("MoveArmScore", new ScoreHighCone(m_arm, false));
        eventMap.put("OpenClaw", new OpenClaw(m_claw, true));
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
