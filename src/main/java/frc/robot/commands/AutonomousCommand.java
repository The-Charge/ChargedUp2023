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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MagicArm;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
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
        //This checks if a path is selected
        if (RobotContainer.getInstance().getSelectedPath() != "None") {
            m_autonomousCommand = getAutonomousCommandPath();
        }

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    public Command getAutonomousCommandPath() {
        getSmartDashValues();

        /**
         * This will load the file "Example Path.path" and generate it with a max
         * velocity of 4 m/s and a max acceleration of 3 m/s^2
         */
        PathPlannerTrajectory examplePath = PathPlanner.loadPath(
                RobotContainer.getInstance().getSelectedPath(),
                new PathConstraints(kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared));

        HashMap<String, Command> eventMap = new HashMap<>();

        // Scores Current game piece
        eventMap.put("MoveArmScore", new ScoreHighCone(m_arm, true));
        eventMap.put("OpenClaw", new OpenClaw(m_claw, m_arm));
        /**
         * Centers gravity for accurate autonomous pathing and
         * Makes next event quicker
         */
        eventMap.put("MoveArmNeutral", new MoveArmToNeutral(m_arm));

        // Picks up game piece
        eventMap.put("MoveArmLow", new MoveMagicArmToXY(m_arm, 1.0, 0.1, 0));
        eventMap.put("CloseClaw", new CloseClaw(m_claw));

        /**
         * Centers gravity for accurate autonomous pathing and
         * Makes next event quicker again
         */
        eventMap.put("MoveArmNeutral", new MoveArmToNeutral(m_arm));

        // Scores previously grabbed piece
        eventMap.put("MoveArmScore", new ScoreHighCone(m_arm, true));
        eventMap.put("OpenClaw", new OpenClaw(m_claw, m_arm));




        m_driveTrain.resetOdometry(new Pose2d());
        /**
         * Create the AutoBuilder. This only needs to be created once when robot code
         * starts, not every time you want to create an auto command. A good place to
         * put this is in RobotContainer along with your subsystems.
         */
        RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
                m_driveTrain::getPose,
                m_driveTrain::resetOdometry,
                new RamseteController(Constants.SysIDConstants.kRamseteB, Constants.SysIDConstants.kRamseteZeta),
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
    
    private void getSmartDashValues() {
        //get Volt constraints for SysID
        ksVolts = SmartDashboard.getNumber("ksVolts", Constants.SysIDConstants.ksVolts);
        kvVoltSecondsPerMeter = SmartDashboard.getNumber("kvVoltSecondsPerMeter", 
        Constants.SysIDConstants.kvVoltSecondsPerMeter);
        kaVoltSecondsSquaredPerMeter = SmartDashboard.getNumber("kaVoltSecondsSquaredPerMeter",
                Constants.SysIDConstants.kaVoltSecondsSquaredPerMeter);

        //get PID values for SysID
        kPDriveVel = SmartDashboard.getNumber("kPDriveVel", Constants.SysIDConstants.kPDriveVel);
        kIDriveVel = SmartDashboard.getNumber("kIDriveVel", Constants.SysIDConstants.kIDriveVel);
        kDDriveVel = SmartDashboard.getNumber("kDDriveVel", Constants.SysIDConstants.kDDriveVel);

        //get max speed and acceleration for SysID
        kMaxSpeedMetersPerSecond = SmartDashboard.getNumber("kMaxSpeedMetersPerSecond",
                Constants.SysIDConstants.kMaxSpeedMetersPerSecond);
        kMaxAccelerationMetersPerSecondSquared = SmartDashboard.getNumber("kMaxAccelerationMetersPerSecondSquared",
                Constants.SysIDConstants.kMaxAccelerationMetersPerSecondSquared);
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
