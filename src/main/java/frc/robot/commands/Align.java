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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Camera;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.Drivetrain;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

public class Align extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private final Drivetrain m_drivetrain;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private final Camera m_camera;
    public double TARGET_HEIGHT_METERS = Units.inchesToMeters(17.3); // Height of target (to be changed)
    final double GOAL_RANGE_METERS = Units.inchesToMeters(14.5 + 25.5); // distance to reach between tag and robot
    public double range, bestTargetYaw;

    // PIDControllers from example: adjust as needed
    private final PIDController forwardController = new PIDController(VisionConstants.LINEAR_P, 0,
            VisionConstants.LINEAR_D);
    private final PIDController turnController = new PIDController(VisionConstants.ANGULAR_P, 0,
            VisionConstants.ANGULAR_D);

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    public Align(Drivetrain subsystem, Camera subsystemCam) {

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_drivetrain = subsystem;
        m_camera = subsystemCam;
        addRequirements(m_drivetrain);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Speeds for arcadedrive
        double forwardSpeed, rotationSpeed;

        // Get result, if there are targets within the cameraview
        var result = m_camera.getFrontCamera().getLatestResult();
        if (result.hasTargets()) {
            // Calculate range between robot and bestTarget
            range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERA_HEIGHT_METERS,
                    TARGET_HEIGHT_METERS, VisionConstants.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
            SmartDashboard.putNumber("Range", range);
            // Calculate and feed speed values into runArcade in Drivetrain
            forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
            rotationSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0);
            bestTargetYaw = Math.abs(result.getBestTarget().getYaw());
            SmartDashboard.putNumber("forward", forwardSpeed);
            SmartDashboard.putNumber("Rotation", rotationSpeed);
            SmartDashboard.putNumber("Target Yaw", result.getBestTarget().getYaw());
            m_drivetrain.runArcade(forwardSpeed, rotationSpeed);
        } else {
            // No targets -> run tankDrive until there is a target in sight
            forwardSpeed = -RobotContainer.getInstance().getrightJoystick().getY();
            rotationSpeed = -RobotContainer.getInstance().getleftJoystick().getY();
            m_drivetrain.run(forwardSpeed, rotationSpeed);
        }
        SmartDashboard.putBoolean("TapeAlign Finished", isFinished());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (bestTargetYaw < 0.5 && Math.abs(GOAL_RANGE_METERS - range) < 0.05) {  
            return true;
        }
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
