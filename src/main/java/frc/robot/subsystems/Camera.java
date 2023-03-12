// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

public class Camera extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
    /**
     * This object is outside the autogernerated code because
     * Robot Builder can't define a Photon Camera
     */
    private PhotonCamera frontCamera;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public Camera() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        frontCamera = new PhotonCamera(VisionConstants.cameraName); // aprilTagCamera
    }

    @Override
    public void periodic() {
        /**
         * This method will be called once per scheduler run
         * Get latest result from camera. If there are targets, put the id of the "best
         * Target" defined by photonlib onto the smartdashboard. No target places -1 on
         * Dash.
         * https://docs.photonvision.org/en/latest/docs/getting-started/pipeline-tuning/reflectiveAndShape/contour-filtering.html#contour-grouping-and-sorting
         */
        var res = frontCamera.getLatestResult();
        if (res.hasTargets()) {
            /**
             * If targets sighted Double range =
             * PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT_METERS,
             * Units.inchesToMeters(21), Constants.CAMERA_PITCH_RADIANS,
             * Units.degreesToRadians(res.getBestTarget().getPitch()));
             */
            double range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERA_HEIGHT_METERS,
                    Units.inchesToMeters(15), VisionConstants.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(res.getBestTarget().getPitch()));

            Transform3d pose = res.getBestTarget().getBestCameraToTarget();

            // Target ID onto smartdash
            SmartDashboard.putNumber("Range", range);
            SmartDashboard.putNumber("X", pose.getX());
            SmartDashboard.putNumber("X adjusted", pose.getX() - Units.inchesToMeters(14.5));
            SmartDashboard.putNumber("Y", pose.getY());
            SmartDashboard.putNumber("Rotation dif", Units.radiansToDegrees(pose.getRotation().getAngle()));
            SmartDashboard.putNumber("Pose Ambiguity", res.getBestTarget().getPoseAmbiguity());
            SmartDashboard.putNumber("Pipeline Index", frontCamera.getPipelineIndex());
        } else {
            // No target found
            SmartDashboard.putNumber("ID", -1);
            SmartDashboard.putNumber("Range", -1);
        }
        SmartDashboard.putBoolean("Has AprilTag Target", frontCamera.getLatestResult().hasTargets());
    }

    // This method will be called once per scheduler run when in simulation
    @Override
    public void simulationPeriodic() {
    }

    /**
     * Put methods for controlling this subsystem
     * Here. Call these from Commands.
     */
    public void setFrontCamera(int index) {
        frontCamera.setPipelineIndex(index);
    }

    public void setFrontCamera() {
        frontCamera.setPipelineIndex(Math.abs(frontCamera.getPipelineIndex() - 1));
        frontCamera.setLED(VisionLEDMode.kDefault);
    }

    public PhotonCamera getFrontCamera() {
        // Return camera object for command manipulation
        return frontCamera;
    }
}
